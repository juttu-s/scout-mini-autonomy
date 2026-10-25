#!/usr/bin/env python3
"""Checks deploy.sh runs inside the live base container, piped in so nothing has to be baked into the images:
    ~/robot_exec.sh base python3 - stationary < deploy_check.py
    ~/robot_exec.sh base python3 - smoke [--nav] < deploy_check.py

stationary  before a deploy: the wheels report no motion, no command source is active in the safety node, and
            move_base has no goal in progress. With localization running it prints "POSE x y yaw" so the restarted
            stack can be initialized where the robot stands.
smoke       after a deploy: topic rates, the safety node leaves FAULT, and (--nav) localization has converged.
            Minimums can be overridden by env (SMOKE_MIN_CMD_HZ=..., SMOKE_MIN_INLIER=...), e.g. to test the rollback.

One PASS/FAIL line per check, measured value vs threshold; exit 0 only if all pass."""
import json
import math
import os
import sys
import time

import rospy

failed = []


def report(ok, what, measured, limit):
    print("%s  %-36s %s  (%s)" % ("PASS" if ok else "FAIL", what, measured, limit), flush=True)
    if not ok:
        failed.append(what)


def published():
    return dict(rospy.get_published_topics())


def collect(topic, cls, seconds):
    """Messages on topic during the next `seconds` (receive times, messages)."""
    got = []
    sub = rospy.Subscriber(topic, cls, lambda m: got.append((time.monotonic(), m)), queue_size=200)
    time.sleep(seconds)
    sub.unregister()
    return got


def stationary():
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    topics = published()
    odom = collect("/odom", Odometry, 2.0)
    if not odom:
        report(False, "/odom publishing", "no messages in 2 s", "needed to see motion")
    else:
        v = max(abs(m.twist.twist.linear.x) for _, m in odom)
        w = max(abs(m.twist.twist.angular.z) for _, m in odom)
        report(v < 0.01 and w < 0.01, "wheels still (/odom)", "max |v| %.3f m/s, |w| %.3f rad/s" % (v, w), "< 0.01")
    states = [json.loads(m.data) for _, m in collect("/scout_safety/state", String, 1.0)]
    if not states:
        report(False, "/scout_safety/state publishing", "no messages in 1 s", "needed to see command sources")
    else:
        sources = sorted({s.get("source") for s in states if s.get("source")})
        report(not sources, "no command source active", ", ".join(sources) or "none", "teleop / nav must be idle")
        out = max(max(abs(x) for x in s.get("cmd_out", [0, 0])) for s in states)
        report(out == 0, "safety output zero", "max |cmd_out| %.3f" % out, "== 0")
    if "/move_base/status" in topics:
        from actionlib_msgs.msg import GoalStatus, GoalStatusArray
        busy = {GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING}
        msgs = collect("/move_base/status", GoalStatusArray, 1.5)
        n = sum(1 for s in (msgs[-1][1].status_list if msgs else []) if s.status in busy)
        report(n == 0, "no move_base goal in progress", "%d active" % n, "== 0")
    if "/localization/odom" in topics:
        loc = collect("/localization/odom", Odometry, 1.5)
        if loc:
            p, q = loc[-1][1].pose.pose.position, loc[-1][1].pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            print("POSE %.3f %.3f %.4f" % (p.x, p.y, yaw), flush=True)


RATES = [  # (env name, topic, minimum Hz): measured on the robot ~ 50 / 50 / 10 / 30 / 23 (docker/README.md)
    ("CMD", "/cmd_vel", 45.0),
    ("ODOM", "/odom", 45.0),
    ("LIDAR", "/livox/lidar", 9.0),
    ("CAMERA", "/camera/camera/color/image_raw", 25.0),
    ("DETECTIONS", "/rs_detector/detections", 15.0),
]


def smoke(nav):
    from std_msgs.msg import String
    counts = {t: [] for _, t, _ in RATES}
    subs = [rospy.Subscriber(t, rospy.AnyMsg, lambda _m, t=t: counts[t].append(time.monotonic()), queue_size=50)
            for _, t, _ in RATES]
    deadline = time.monotonic() + 30.0   # first messages: the camera/detector may still be warming up
    while time.monotonic() < deadline and not all(counts.values()):
        time.sleep(0.2)
    for c in counts.values():
        c.clear()
    time.sleep(5.0)
    for s in subs:
        s.unregister()
    for name, topic, default in RATES:
        lim = float(os.environ.get("SMOKE_MIN_%s_HZ" % name, default))
        hz = len(counts[topic]) / 5.0
        report(hz >= lim, topic + " rate", "%.1f Hz" % hz, ">= %g Hz" % lim)
    state, deadline = None, time.monotonic() + 30.0
    while time.monotonic() < deadline:
        msgs = collect("/scout_safety/state", String, 0.5)
        if msgs:
            state = json.loads(msgs[-1][1].data)
            if state.get("state") != "FAULT":
                break
    report(state is not None and state.get("state") != "FAULT", "safety node out of FAULT",
           "%s (%s)" % (state.get("state"), state.get("reason")) if state else "no state", "OK / SLOW / STOP")
    if nav:
        from hdl_localization.msg import ScanMatchingStatus
        lim = float(os.environ.get("SMOKE_MIN_INLIER", 0.9))
        best, deadline = None, time.monotonic() + 30.0
        while time.monotonic() < deadline:
            msgs = collect("/localization/status", ScanMatchingStatus, 1.0)
            if msgs:
                best = msgs[-1][1].inlier_fraction
                if best >= lim:
                    break
        report(best is not None and best >= lim, "localization inlier fraction",
               "%.3f" % best if best is not None else "no status", ">= %g" % lim)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode not in ("stationary", "smoke"):
        sys.exit("usage: deploy_check.py stationary | smoke [--nav]")
    rospy.init_node("deploy_check", anonymous=True, disable_signals=True)
    stationary() if mode == "stationary" else smoke("--nav" in sys.argv[2:])
    print("%s: %s" % (mode, "all checks passed" if not failed else "FAILED: " + ", ".join(failed)), flush=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
