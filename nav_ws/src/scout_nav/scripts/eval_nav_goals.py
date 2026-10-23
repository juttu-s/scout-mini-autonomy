#!/usr/bin/env python3
"""Repeatable navigation evaluation: send a fixed list of goals through move_base, one CSV row per goal attempt.

    rosrun scout_nav eval_nav_goals.py $(rospack find scout_nav)/config/nav_eval_goals_lab2.yaml \\
        [--csv PATH] [--laps 1] [--from NAME] [--timeout-base 30] [--timeout-per-m 6] [--min-inlier 0.95] \\
        [--operator NAME] [--notes TEXT] [--no-clear] [--dry-run]

Protocol (physical setup, goals, safety): scout_nav/docs/nav_eval_protocol.md. Each goal waits for Enter after the
operator has checked that the route is clear (s + Enter: skip, q + Enter: quit); there is no unattended mode.
Before a goal it refuses to drive while localization is below --min-inlier or safety is in FAULT, then clears
the costmaps (--no-clear to keep them) so stale obstacle marks don't carry over between goals.

Per attempt, logged in the window from sending the goal to its result:
  result      SUCCEEDED / ABORTED / PREEMPTED / REJECTED (move_base), TIMEOUT (cancelled after
              timeout-base + timeout-per-m * straight distance), CANCELLED (Ctrl+C), SKIPPED
  pose error  final /localization/odom pose against the goal, xy (m) and yaw (deg)
  time        goal sent -> result
  path        driven length (steps >= 2 cm of /localization/odom) and straight start -> goal distance
  recoveries  /move_base/recovery_status (count and names)
  errors      /rosout_agg >= WARN: TF errors from any node; move_base planner and control failures (counted apart)
  safety      /scout_safety/state messages per state; minimum localization inlier fraction
CSV: --csv, default ~/maps/<map>/nav_eval/nav_eval.csv (the same file from the host or the base container). Rows are
appended across sessions; an existing file with different columns is refused, never mixed.
"""
import argparse
import csv
import datetime
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from collections import Counter

import actionlib
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped with tf2)
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from hdl_localization.msg import ScanMatchingStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, RecoveryStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

COLUMNS = ["session_id", "start_time", "map", "git_rev", "operator", "lap", "goal_name", "goal_x", "goal_y",
           "goal_yaw_deg", "start_x", "start_y", "start_yaw_deg", "result", "status_text", "time_s", "final_x", "final_y",
           "final_yaw_deg", "err_xy_m", "err_yaw_deg", "path_len_m", "straight_m", "path_ratio", "recoveries_n",
           "recoveries", "tf_errors_n", "tf_error_first", "planner_errors_n", "control_errors_n", "safety_ok",
           "safety_slow", "safety_stop", "safety_fault", "min_inlier", "costmaps_cleared", "notes"]
RESULTS = {GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED", GoalStatus.PREEMPTED: "PREEMPTED",
           GoalStatus.REJECTED: "REJECTED", GoalStatus.RECALLED: "RECALLED"}
TF_ERR = re.compile(r"Extrapolation|Lookup would require|TF_|transform|tf2", re.I)
PLANNER_ERR = re.compile(r"NO PATH|Failed to get a plan")
CONTROL_ERR = re.compile(r"valid control|DWA planner failed")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def git_rev(path):
    try:
        rev = subprocess.run(["git", "-C", path, "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                             timeout=5).stdout.strip()
        dirty = subprocess.run(["git", "-C", path, "status", "--porcelain", "--", "."], capture_output=True, text=True,
                               timeout=5).stdout.strip()
        return (rev + ("-dirty" if dirty else "")) or "n/a"
    except (OSError, subprocess.SubprocessError):
        return "n/a"


def open_csv(path):
    """Append handle; header written for a new file, refused on a column mismatch."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path) and os.path.getsize(path) > 0:
        with open(path, newline="") as f:
            header = next(csv.reader(f), [])
        if header != COLUMNS:
            raise SystemExit("%s has different columns than this script writes; refusing to mix them.\n"
                             "Use --csv for a new file (or rename the old one)." % path)
        f = open(path, "a", newline="")
        return f, csv.DictWriter(f, COLUMNS)
    f = open(path, "w", newline="")
    w = csv.DictWriter(f, COLUMNS)
    w.writeheader()
    f.flush()
    return f, w


class Monitor:
    """Everything observed during one goal window (start() .. stop())."""

    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.odom = None            # latest /localization/odom
        self.inlier = None          # latest inlier fraction
        self.safety = None          # latest safety state
        self.reset()
        rospy.Subscriber("/localization/odom", Odometry, self._odom, queue_size=50)
        rospy.Subscriber("/localization/status", ScanMatchingStatus, self._status, queue_size=10)
        rospy.Subscriber("/scout_safety/state", String, self._safety, queue_size=100)
        rospy.Subscriber("/move_base/recovery_status", RecoveryStatus, self._recovery, queue_size=10)
        rospy.Subscriber("/rosout_agg", Log, self._log, queue_size=500)

    def reset(self):
        with self.lock:
            self.path, self.last_pt = 0.0, None
            self.recoveries, self.tf_errors = [], []
            self.planner_errors = self.control_errors = 0
            self.safety_counts, self.min_inlier = Counter(), None

    def start(self):
        self.reset()
        self.active = True

    def stop(self):
        self.active = False

    def _odom(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.odom = m
            if self.active:
                if self.last_pt is None:
                    self.last_pt = (p.x, p.y)
                d = math.hypot(p.x - self.last_pt[0], p.y - self.last_pt[1])
                if d >= 0.02:  # standing jitter doesn't add length
                    self.path += d
                    self.last_pt = (p.x, p.y)

    def _status(self, m):
        with self.lock:
            self.inlier = m.inlier_fraction
            if self.active:
                self.min_inlier = m.inlier_fraction if self.min_inlier is None else min(self.min_inlier, m.inlier_fraction)

    def _safety(self, m):
        try:
            s = json.loads(m.data)["state"]
        except (ValueError, KeyError):
            return
        with self.lock:
            self.safety = s
            if self.active:
                self.safety_counts[s] += 1

    def _recovery(self, m):
        if self.active:
            with self.lock:
                self.recoveries.append(m.recovery_behavior_name)

    def _log(self, m):
        if not self.active or m.level < Log.WARN:
            return
        with self.lock:
            if TF_ERR.search(m.msg):
                self.tf_errors.append("%s: %s" % (m.name, m.msg[:150]))
            if m.name == "/move_base":
                self.planner_errors += bool(PLANNER_ERR.search(m.msg))
                self.control_errors += bool(CONTROL_ERR.search(m.msg))


def pose_in(buf, frame):
    """(x, y, yaw) of base_link in frame, or None."""
    try:
        t = buf.lookup_transform(frame, "base_link", rospy.Time(0), rospy.Duration(1.0)).transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None


def ready(mon, min_inlier):
    """None if it's fine to drive, else the reason not to."""
    if mon.inlier is None:
        return "no /localization/status yet"
    if mon.inlier < min_inlier:
        return "localization inliers %.2f < %.2f: set the pose in Foxglove (2D Pose Estimate)" % (mon.inlier, min_inlier)
    if mon.safety is None:
        return "no /scout_safety/state yet"
    if mon.safety == "FAULT":
        return "safety is in FAULT"
    return None


def dry_run(goals, frame, buf, mon):
    """Checks and make_plan for every leg; sends nothing, writes nothing."""
    print("localization inliers: %s | safety: %s | ready to drive: %s" % (mon.inlier, mon.safety, ready(mon, 0.95) or "yes"))
    mk = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
    try:
        mk.wait_for_service(5)
    except rospy.ROSException:
        raise SystemExit("/move_base/make_plan not available: is navigation running?")
    # make_plan doesn't transform: GlobalPlanner rejects poses that aren't in the costmap's global frame (map_floor
    # here), while the move_base action used for real goals converts them itself
    gframe = rospy.get_param("/move_base/global_costmap/global_frame", frame)
    here = pose_in(buf, frame)
    legs = ([("robot", here[0], here[1])] if here else []) + [(g["name"], g["x"], g["y"]) for g in goals]
    legs.append((goals[0]["name"], goals[0]["x"], goals[0]["y"]))  # the lap closes at the first goal
    try:
        cm = rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid, 5)
    except rospy.ROSException:
        cm = None
    for (na, xa, ya), (nb, xb, yb) in zip(legs, legs[1:]):
        s, g = PoseStamped(), PoseStamped()
        for p, x, y in ((s, xa, ya), (g, xb, yb)):
            p.header.frame_id, p.pose.position.x, p.pose.position.y, p.pose.orientation.w = frame, x, y, 1.0
        s, g = (buf.transform(p, gframe, rospy.Duration(1.0)) for p in (s, g))
        try:
            plan = mk(start=s, goal=g, tolerance=0.0).plan.poses
        except rospy.ServiceException as e:
            plan, note = [], str(e)
        xy = np.array([[q.pose.position.x, q.pose.position.y] for q in plan])
        L = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else None
        cost = ""
        if cm is not None:
            r = int((g.pose.position.y - cm.info.origin.position.y) / cm.info.resolution)
            c = int((g.pose.position.x - cm.info.origin.position.x) / cm.info.resolution)
            v = cm.data[r * cm.info.width + c]
            cost = ", goal cell cost %d%s" % (v, " (LETHAL: obstacle or stale mark; real runs clear first)" if v >= 99 else "")
        print("  %-12s -> %-12s straight %5.2f m, plan %s%s" % (na, nb, math.hypot(xb - xa, yb - ya),
                                                              "%.2f m" % L if L else "NONE", cost))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("goals")
    ap.add_argument("--csv")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--from", dest="start_at", help="begin the first lap at this goal name")
    ap.add_argument("--timeout-base", type=float, default=30.0)
    ap.add_argument("--timeout-per-m", type=float, default=6.0)
    ap.add_argument("--min-inlier", type=float, default=0.95)
    ap.add_argument("--operator", default=os.environ.get("USER", ""))
    ap.add_argument("--notes", default="")
    ap.add_argument("--no-clear", action="store_true",
                    help="don't clear the costmaps before each goal (default: clear, so stale marks from earlier goals "
                         "or a mislocalized spell don't decide the result; obstacles in view are re-marked in one scan)")
    ap.add_argument("--dry-run", action="store_true", help="check localization and plan every leg; no goals sent")
    a = ap.parse_args(rospy.myargv()[1:])

    cfg = yaml.safe_load(open(a.goals))
    goals, frame, mapname = cfg["goals"], cfg.get("frame", "map"), cfg.get("map", "unknown")
    names = [g["name"] for g in goals]
    if a.start_at and a.start_at not in names:
        raise SystemExit("--from %s: not in %s" % (a.start_at, names))

    rospy.init_node("eval_nav_goals", disable_signals=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    mon = Monitor()
    rospy.sleep(2.0)
    if a.dry_run:
        dry_run(goals, frame, buf, mon)
        return

    path = os.path.expanduser(a.csv or "~/maps/%s/nav_eval/nav_eval.csv" % mapname)
    f, writer = open_csv(path)
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(10)):
        raise SystemExit("move_base action server not available: is navigation running (~/robot_up.sh --nav %s)?" % mapname)
    session = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rev = git_rev(os.path.dirname(os.path.realpath(__file__)))
    print("session %s, %d goals x %d laps -> %s" % (session, len(goals), a.laps, path))

    # A lap starts at the start goal (where the robot stands) and visits the others in order, ending back at it.
    start = names.index(cfg.get("start", names[0]))
    circuit = goals[start + 1:] + goals[:start + 1]
    seq = []
    for lap in range(1, a.laps + 1):
        order = circuit
        if lap == 1 and a.start_at:
            order = circuit[[g["name"] for g in circuit].index(a.start_at):]
        seq += [(lap, g) for g in order]
    rows = []
    try:
        for lap, g in seq:
            gx, gy, gyaw = float(g["x"]), float(g["y"]), math.radians(float(g["yaw_deg"]))
            while True:
                why = ready(mon, a.min_inlier)
                if not why:
                    break
                print("  waiting: %s" % why)
                time.sleep(2.0)
            here = pose_in(buf, frame)
            straight = math.hypot(gx - here[0], gy - here[1]) if here else float("nan")
            timeout = a.timeout_base + a.timeout_per_m * (straight if here else 10.0)
            ans = input("\nlap %d: next %s (%.2f, %.2f, %.0f deg), %.1f m away, timeout %.0f s. "
                        "Route clear? Enter = go, s = skip, q = quit: " % (lap, g["name"], gx, gy, g["yaw_deg"],
                                                                         straight, timeout)).strip().lower()
            row = {"session_id": session, "map": mapname, "git_rev": rev, "operator": a.operator, "lap": lap,
                   "goal_name": g["name"], "goal_x": gx, "goal_y": gy, "goal_yaw_deg": g["yaw_deg"], "notes": a.notes,
                   "start_time": datetime.datetime.now().isoformat(timespec="seconds")}
            if here:
                row.update(start_x=round(here[0], 3), start_y=round(here[1], 3), start_yaw_deg=round(math.degrees(here[2]), 1),
                           straight_m=round(straight, 3))
            if ans == "q":
                break
            if ans == "s":
                row.update(result="SKIPPED")
                writer.writerow(row)
                f.flush()
                rows.append(row)
                continue

            if not a.no_clear:
                try:
                    rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
                    time.sleep(1.0)  # let the sensors re-mark what is really there
                except rospy.ServiceException as e:
                    print("  clear_costmaps failed: %s" % e)
            row["costmaps_cleared"] = int(not a.no_clear)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id, goal.target_pose.header.stamp = frame, rospy.Time.now()
            goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = gx, gy
            goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
            mon.start()
            t0 = time.time()
            client.send_goal(goal)
            result = None
            try:
                while result is None:
                    if client.wait_for_result(rospy.Duration(0.2)):
                        st = client.get_state()
                        result = RESULTS.get(st, "STATE_%d" % st)
                    elif time.time() - t0 > timeout:
                        client.cancel_goal()
                        client.wait_for_result(rospy.Duration(5))
                        result = "TIMEOUT"
            except KeyboardInterrupt:
                client.cancel_goal()
                client.wait_for_result(rospy.Duration(5))
                result = "CANCELLED"
            dt = time.time() - t0
            rospy.sleep(0.5)  # last poses / log lines of the window
            mon.stop()

            row.update(result=result, time_s=round(dt, 2), status_text=client.get_goal_status_text()[:200])
            with mon.lock:
                od = mon.odom
                row.update(path_len_m=round(mon.path, 3), recoveries_n=len(mon.recoveries),
                           recoveries=";".join(mon.recoveries), planner_errors_n=mon.planner_errors,
                           control_errors_n=mon.control_errors, safety_ok=mon.safety_counts["OK"],
                           safety_slow=mon.safety_counts["SLOW"], safety_stop=mon.safety_counts["STOP"],
                           safety_fault=mon.safety_counts["FAULT"],
                           min_inlier=round(mon.min_inlier, 4) if mon.min_inlier is not None else "")
                tf_errors = list(mon.tf_errors)
            if pose_in(buf, frame) is None:
                tf_errors.append("eval_nav_goals: map -> base_link lookup failed at the end of the goal")
            row.update(tf_errors_n=len(tf_errors), tf_error_first=tf_errors[0] if tf_errors else "")
            if od is not None:
                p = od.pose.pose.position
                fyaw = yaw_of(od.pose.pose.orientation)
                row.update(final_x=round(p.x, 3), final_y=round(p.y, 3), final_yaw_deg=round(math.degrees(fyaw), 1),
                           err_xy_m=round(math.hypot(p.x - gx, p.y - gy), 3),
                           err_yaw_deg=round(abs(math.degrees(wrap(fyaw - gyaw))), 1))
            if here and straight > 0.05:
                row["path_ratio"] = round(mon.path / straight, 3)
            writer.writerow(row)
            f.flush()
            rows.append(row)
            print("  %s after %.1f s: error %s m / %s deg, path %.2f m (straight %.2f), recoveries %d, TF errors %d, "
                  "planner/control errors %d/%d, safety %s" % (
                      result, dt, row.get("err_xy_m", "?"), row.get("err_yaw_deg", "?"), mon.path, straight,
                      row["recoveries_n"], row["tf_errors_n"], row["planner_errors_n"], row["control_errors_n"],
                      dict(mon.safety_counts)))
            if result == "CANCELLED":
                break
    except KeyboardInterrupt:
        print("\ninterrupted (no goal active)")
    finally:
        f.close()
    done = [r for r in rows if r.get("result") not in ("SKIPPED", None)]
    ok = [r for r in done if r["result"] == "SUCCEEDED"]
    print("\nsession %s: %d attempts, %d succeeded%s -> %s" % (
        session, len(done), len(ok),
        ", mean error %.2f m / %.0f deg" % (np.mean([r["err_xy_m"] for r in ok]), np.mean([r["err_yaw_deg"] for r in ok]))
        if ok and all("err_xy_m" in r for r in ok) else "", path))
    print("Robot stops: move_base has no active goal; safety drops /nav/cmd_vel after 0.5 s without commands.")
    sys.exit(0)


if __name__ == "__main__":
    main()
