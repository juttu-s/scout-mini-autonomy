#!/usr/bin/env python3
"""Offline scenario tests for scout_safety/safety_node.py, no camera or robot needed.

    test/run_scenario.sh <scenario id>|--all <out dir>    # starts roscore, the node with config/safety.yaml, a bag
    python3 test/scenario_test.py --list                  # scenario ids as JSON (no ROS needed)
    python3 test/scenario_test.py --scenario ID [--json PATH]

Each scenario runs against a freshly started node (run_scenario.sh) and, except no_detector_fault, starts from the
same prime: detector alive with nobody in view, teleop at (0.4, 0.3), wait until the node has recovered to OK. So
none depends on another having run first; checks that only make sense in sequence are phases of one scenario.
Every assertion is recorded as {what, measured, expected, tolerance|threshold, passed}; thresholds come from the
node's live parameters (/scout_safety/*), i.e. from the config under test. Exit code 1 if any assertion failed.
Only run it against its own master (run_scenario.sh refuses when one is already running): the node publishes /cmd_vel.
"""
import argparse
import json
import sys
import time
import traceback

# Scenario table: (id, name, phases). A phase is ("check", name, wait, changes, state, lin, ang, tol) or a
# special-phase name handled in run(). Positions are base_link (x forward, y left); tele/nav are (lin, ang).
NAV = dict(tele=None, nav=(0.3, 0.0))
SCENARIOS = [
    ("no_detector_fault", "no detector output -> FAULT", [
        ("check", "no detector output -> FAULT", 1.0, dict(tele=(0.4, 0.3)), "FAULT", 0.0, 0.0, 0.02)]),
    ("detector_alive_full_speed", "detector alive, nobody -> full speed", []),  # the prime is the check
    ("person_far_full_speed", "person 3 m ahead -> full speed", [
        ("check", "person 3 m ahead -> full speed", 0.5, dict(people=[(3.31, 0.0)]), "OK", 0.4, None, 0.02)]),
    ("person_mid_scaled", "person 1.5 m ahead -> scaled", [
        ("check", "person 1.5 m ahead -> scaled", 0.5, dict(people=[(1.81, 0.0)]), "SLOW", 0.25, None, 0.04)]),
    ("stop_close_turn_and_reverse", "person 0.6 m ahead -> stop, turning and slow reverse allowed", [
        ("check", "person 0.6 m ahead -> stop, turning ok", 0.5, dict(people=[(0.91, 0.0)]), "STOP", 0.0, 0.3, 0.02),
        ("check", "reverse away while stopped -> capped -0.2", 0.5, dict(tele=(-0.5, 0.0)), "STOP", -0.2, None, 0.02)]),
    ("person_beside_no_limit", "person beside the path -> no limit", [
        ("check", "person beside the path -> no limit", 1.0, dict(people=[(1.81, 1.0)]), "OK", 0.4, None, 0.02)]),
    ("person_adjacent_no_turn", "person next to robot -> no turning either", [
        ("check", "person next to robot -> no turning either", 0.5, dict(people=[(0.5, 0.3)]), "STOP", 0.0, 0.0,
         0.02)]),
    ("teleop_priority_then_nav_fallback", "teleop overrides nav, nav takes over when teleop goes silent", [
        ("check", "nav 0.3 + teleop 0.1 -> teleop wins", 1.0, dict(nav=(0.3, 0.0), tele=(0.1, 0.0)), "OK", 0.1, None,
         0.02),
        ("check", "teleop goes silent -> nav after timeout", 1.0, dict(tele=None), "OK", 0.3, None, 0.02)]),
    ("big_box_no_depth_stop", "big person box without depth -> stop", [
        ("check", "big person box without depth -> stop", 0.5, dict(nodepth=[0.62]), "STOP", 0.0, None, 0.02)]),
    ("small_box_no_depth_slow", "small person box without depth -> slow", [
        ("check", "small person box without depth -> slow", 1.0, dict(nodepth=[0.3]), "SLOW", 0.1, None, 0.02)]),
    ("ttc_stop_hold_release", "person running at the robot -> TTC stop, held for clear_time, then released", [
        ("check", "on nav", 1.0, NAV, "OK", 0.3, None, 0.02),
        ("check", "person 3 m ahead running at robot -> TTC stop", 0.5,
         dict(people=[(3.31, 0.0)], tracks=[(3.31, 0.0, -2.0, 0.0)]), "STOP", 0.0, None, 0.02),
        ("check", "STOP is held for clear_time after clearing", 0.25, dict(people=[], tracks=[]), "STOP", 0.0, None,
         0.02),
        ("check", "...then released", 0.6, {}, "OK", 0.3, None, 0.02)]),
    ("person_aside_no_ttc", "person standing 1.5 m to the side, not on path -> no TTC stop", [
        ("check", "person 1.5 m to the side, standing -> no TTC stop", 1.0,
         dict(NAV, people=[(3.0, 1.5)], tracks=[(3.0, 1.5, 0.0, 0.0)]), "OK", 0.3, None, 0.02)]),
    ("flicker_steady_slow", "person flickering in/out of the corridor -> steady SLOW, never back to OK", [
        ("check", "person flickering in/out of the corridor -> steady SLOW", 1.0, dict(NAV, flicker=True), "SLOW",
         0.25, None, 0.04),
        "never_ok_while_flickering"]),
    ("detector_stall_fault", "detector stops publishing mid-run -> zero-output FAULT within the staleness window",
     ["detector_stall"]),
    ("rogue_publisher_reported", "rogue /cmd_vel publisher is reported", ["rogue_publisher"]),
]
IDS = [s[0] for s in SCENARIOS]


class Run:
    def __init__(self, sid):
        global rospy, np, Twist, String
        import numpy as np
        import rospy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import CameraInfo
        from std_msgs.msg import String
        from vision_msgs.msg import (Detection2D, Detection2DArray, Detection3D, Detection3DArray,
                                     ObjectHypothesisWithPose)
        from rs_object_detection.msg import TrackedObject, TrackedObjectArray
        self.msg = dict(Detection2D=Detection2D, Detection2DArray=Detection2DArray, Detection3D=Detection3D,
                        Detection3DArray=Detection3DArray, Hyp=ObjectHypothesisWithPose, TrackedObject=TrackedObject,
                        TrackedObjectArray=TrackedObjectArray)
        self.sid, self.assertions = sid, []
        rospy.init_node("safety_test")
        self.state0 = rospy.wait_for_message("/scout_safety/state", String, 15.0)  # node is up
        self.params = rospy.get_param("/scout_safety", {})
        self.K = np.array([[600, 0, 424], [0, 600, 240], [0, 0, 1.0]])
        self.pub3 = rospy.Publisher("/rs_detector/detections", Detection3DArray, queue_size=1)
        self.pub2 = rospy.Publisher("/rs_detector/detections_2d", Detection2DArray, queue_size=1)
        self.pubt = rospy.Publisher("/rs_detector/tracks", TrackedObjectArray, queue_size=1)
        pubi = rospy.Publisher("/camera/camera/color/camera_info", CameraInfo, queue_size=1, latch=True)
        self.tele = rospy.Publisher("/teleop/cmd_vel", Twist, queue_size=1)
        self.nav = rospy.Publisher("/nav/cmd_vel", Twist, queue_size=1)
        pubi.publish(CameraInfo(header=rospy.Header(frame_id="camera_color_optical_frame"), height=480, width=848,
                                K=list(self.K.ravel())))
        self.out, self.state = None, None
        self.cmd_log, self.state_log = [], []  # (receipt time, ...) for the timing scenarios
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_cb)
        rospy.Subscriber("/scout_safety/state", String, self._state_cb)
        self.sc = dict(detect=False, people=[], nodepth=[], tracks=[], tele=None, nav=None, flicker=False)
        self.last_det_stamp = None
        rospy.Timer(rospy.Duration(1 / 30.0), self.tick)
        time.sleep(1.0)

    def _cmd_cb(self, m):
        self.out = m
        self.cmd_log.append((rospy.get_time(), m.linear.x, m.angular.z))

    def _state_cb(self, m):
        self.state = json.loads(m.data)
        self.state_log.append((rospy.get_time(), self.state["state"]))

    def p(self, name):
        return self.params[name]

    @staticmethod
    def base_to_opt(x, y, z=0.0):  # base_link point -> camera optical (camera at x=0.3, z=0.2, facing forward)
        return np.array([-(y - 0.0), -(z - 0.2), x - 0.3])

    def twist(self, v):
        t = Twist()
        t.linear.x, t.angular.z = v
        return t

    def tick(self, _):
        sc, M = self.sc, self.msg
        now = rospy.Time.now()
        if sc["tele"] is not None:
            self.tele.publish(self.twist(sc["tele"]))
        if sc["nav"] is not None:
            self.nav.publish(self.twist(sc["nav"]))
        if not sc["detect"]:
            return
        if sc["flicker"]:  # person jumps in and out of the corridor every 0.1 s
            sc["people"] = [(1.81, 0.0)] if int(time.time() * 10) % 2 else [(1.81, 1.0)]
        h = rospy.Header(stamp=now, frame_id="camera_color_optical_frame")
        a3, a2 = M["Detection3DArray"](header=h), M["Detection2DArray"](header=h)
        for (x, y) in sc["people"]:
            q = self.base_to_opt(x, y, 0.9)
            d = M["Detection3D"](header=h)
            d.results.append(M["Hyp"](id=0, score=0.9))
            d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = q
            a3.detections.append(d)
            d2 = M["Detection2D"](header=h)
            d2.results.append(M["Hyp"](id=0, score=0.9))
            d2.bbox.center.x = self.K[0, 0] * q[0] / q[2] + self.K[0, 2]
            d2.bbox.center.y = self.K[1, 1] * q[1] / q[2] + self.K[1, 2]
            d2.bbox.size_y = 200
            a2.detections.append(d2)
        for hfrac in sc["nodepth"]:
            d2 = M["Detection2D"](header=h)
            d2.results.append(M["Hyp"](id=0, score=0.9))
            d2.bbox.center.x, d2.bbox.center.y, d2.bbox.size_y = 100, 240, hfrac * 480
            a2.detections.append(d2)
        ta = M["TrackedObjectArray"]()
        ta.header.stamp, ta.header.frame_id = now, "base_link"
        for i, (x, y, vx, vy) in enumerate(sc["tracks"]):
            o = M["TrackedObject"](id=i + 1, class_id=0, has_position=True)
            o.position.x, o.position.y = x, y
            o.velocity.x, o.velocity.y = vx, vy
            ta.objects.append(o)
        self.pub3.publish(a3)
        self.pub2.publish(a2)
        self.pubt.publish(ta)
        self.last_det_stamp = now.to_sec()

    # --- assertions -------------------------------------------------------------------------------------------
    def expect(self, phase, what, measured, expected, tolerance=None, threshold=None, passed=None):
        if passed is None:
            passed = measured == expected if tolerance is None else abs(measured - expected) <= tolerance
        a = dict(phase=phase, what=what, measured=measured, expected=expected, passed=bool(passed))
        if tolerance is not None:
            a["tolerance"] = tolerance
        if threshold is not None:
            a["threshold"] = threshold
        self.assertions.append(a)
        return a["passed"]

    def check(self, name, wait, changes, exp_state, exp_lin, exp_ang, tol):
        self.sc.update(changes)
        time.sleep(wait)
        o, st = self.out, self.state
        ok = self.expect(name, "state", st["state"], exp_state)
        ok &= self.expect(name, "linear.x (m/s)", round(o.linear.x, 3), exp_lin, tolerance=tol)
        if exp_ang is not None:
            ok &= self.expect(name, "angular.z (rad/s)", round(o.angular.z, 3), exp_ang, tolerance=tol)
        print("%-4s %-52s state %-5s lin %+.2f ang %+.2f  (want %s %+.2f%s)  %s" % (
            "PASS" if ok else "FAIL", name, st["state"], o.linear.x, o.angular.z, exp_state, exp_lin,
            "" if exp_ang is None else " ang %+.2f" % exp_ang, st["reason"]))

    def prime(self):
        self.check("prime: detector alive, nobody -> full speed", self.p("recover_time") + 0.6,
                   dict(detect=True, tele=(0.4, 0.3)), "OK", 0.4, 0.3, 0.02)

    # --- special phases ---------------------------------------------------------------------------------------
    def never_ok_while_flickering(self):
        n0 = len(self.state_log)
        time.sleep(1.0)
        states = [s for _, s in self.state_log[n0:]]
        non_slow = sorted(set(s for s in states if s != "SLOW"))
        ok = self.expect("...and it never toggles back to OK", "state messages in 1 s", len(states), "> 10",
                         passed=len(states) > 10)
        ok &= self.expect("...and it never toggles back to OK", "states other than SLOW", non_slow, [])
        print("%-4s %-52s %d state msgs, other than SLOW: %s" % ("PASS" if ok else "FAIL",
              "...and it never toggles back to OK", len(states), non_slow))

    def detector_stall(self):
        """Stop publishing detections mid-run: FAULT within max_detection_age (+ two node cycles), zero output for
        as long as it lasts while teleop keeps commanding, and recovery only after recover_time."""
        mda, rate, rec = self.p("max_detection_age"), self.p("rate"), self.p("recover_time")
        ph = "before the stall"
        ok = self.expect(ph, "output while healthy is non-zero, linear.x (m/s)", round(self.out.linear.x, 3), 0.4,
                         tolerance=0.02)
        # 1. stall
        n_state = len(self.state_log)
        self.sc["detect"] = False
        time.sleep(0.1)  # a tick already past its detect check finishes publishing
        t_last = self.last_det_stamp
        deadline = time.time() + mda + 2.0
        t_fault = None
        while time.time() < deadline and t_fault is None:
            t_fault = next((t for t, s in self.state_log[n_state:] if s == "FAULT"), None)
            time.sleep(0.01)
        ph = "detections stop"
        limit = round(mda + 2.0 / rate + 0.05, 3)
        latency = None if t_fault is None else round(t_fault - t_last, 3)
        ok &= self.expect(ph, "last detection stamp -> first FAULT state (s)", latency, "<= %.3f" % limit,
                          threshold=limit, passed=latency is not None and latency <= limit)
        # 2. zero output while stalled (teleop still commanding 0.4 / 0.3)
        if t_fault is not None:
            time.sleep(1.0)
            cmds = [c for c in self.cmd_log if t_fault <= c[0] <= t_fault + 1.0]
            states = [s for t, s in self.state_log if t_fault <= t <= t_fault + 1.0]
            peak = max([max(abs(lin), abs(ang)) for _, lin, ang in cmds] or [None])
            ph = "1 s after FAULT"
            ok &= self.expect(ph, "/cmd_vel messages", len(cmds), ">= %d" % int(0.8 * rate),
                              passed=len(cmds) >= int(0.8 * rate))
            ok &= self.expect(ph, "max |linear.x|, |angular.z| on /cmd_vel", peak, 0.0, passed=peak == 0.0)
            ok &= self.expect(ph, "states other than FAULT", sorted(set(states) - {"FAULT"}), [])
        # 3. recovery: FAULT ("recovering") for recover_time, then OK
        n_state = len(self.state_log)
        t_resume = rospy.get_time()
        self.sc["detect"] = True
        deadline = time.time() + rec + 2.0
        t_ok = None
        while time.time() < deadline and t_ok is None:
            t_ok = next((t for t, s in self.state_log[n_state:] if s != "FAULT"), None)
            time.sleep(0.01)
        rtime = None if t_ok is None else round(t_ok - t_resume, 3)
        lo, hi = round(rec - 0.05, 3), round(rec + 0.3, 3)
        ok &= self.expect("detections resume", "resume -> first non-FAULT state (s)", rtime,
                          "%.3f .. %.3f" % (lo, hi), threshold=[lo, hi],
                          passed=rtime is not None and lo <= rtime <= hi)
        time.sleep(0.3)
        ok &= self.expect("detections resume", "state after recovery", self.state["state"], "OK")
        print("%-4s %-52s FAULT %s s after the last detection (limit %.3f), %s s to recover (%.2f..%.2f)" % (
            "PASS" if ok else "FAIL", "detector stalls -> zero-output FAULT", latency, limit, rtime, lo, hi))

    def rogue_publisher(self):
        rogue = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # a node driving the robot around the safety layer
        time.sleep(2.6)
        bypass = self.state.get("bypass") or []
        ok = self.expect("rogue publisher", "bypass list contains /safety_test", bypass, ["/safety_test"],
                         passed="/safety_test" in bypass)
        print("%-4s %-52s bypass reported: %s" % ("PASS" if ok else "FAIL", "rogue /cmd_vel publisher is reported",
                                                  bypass))
        rogue.unregister()

    def run(self):
        _, name, phases = SCENARIOS[IDS.index(self.sid)]
        if self.sid != "no_detector_fault":
            self.prime()
        for ph in phases:
            if isinstance(ph, str):
                getattr(self, ph)()
            else:
                self.check(*ph[1:])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="print the scenario ids as a JSON list")
    ap.add_argument("--scenario", choices=IDS)
    ap.add_argument("--json", help="write the result here")
    a = ap.parse_args()
    if a.list:
        print(json.dumps(IDS))
        return
    if not a.scenario:
        ap.error("--list or --scenario ID")
    name = SCENARIOS[IDS.index(a.scenario)][1]
    result = dict(scenario=a.scenario, name=name, passed=False, assertions=[], params={}, error=None)
    try:
        r = Run(a.scenario)
        result["params"] = r.params
        try:
            r.run()
        finally:
            result["assertions"] = r.assertions
        result["passed"] = bool(r.assertions) and all(x["passed"] for x in r.assertions)
    except Exception:
        result["error"] = traceback.format_exc()
        print(result["error"], file=sys.stderr)
    n_ok = sum(x["passed"] for x in result["assertions"])
    print("%s: %s (%d/%d assertions passed)" % (a.scenario, "PASS" if result["passed"] else "FAIL", n_ok,
                                                 len(result["assertions"])))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(result, f, indent=2)
    sys.exit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
