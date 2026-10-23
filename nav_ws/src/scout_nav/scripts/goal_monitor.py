#!/usr/bin/env python3
"""Watch one move_base goal and report how it went.

    rosrun scout_nav goal_monitor.py [--timeout 900]

Waits for the next goal, then reports: result, drive phase (distance, mean speed, turn reversals), final phase
within 0.3 m (time turning, reversals, degrees turned), final position/heading error, safety states, and move_base
warnings/errors.
"""
import argparse
import json
import time
from collections import Counter

import numpy as np
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from rosgraph_msgs.msg import Log
from std_msgs.msg import String


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def flips(w):
    w = w[np.abs(w) > 0.05]
    return int((np.diff(np.sign(w)) != 0).sum()) if len(w) > 1 else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=900.0)
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("goal_monitor", anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    st = {"goal": None, "t0": None, "status": None}
    cmd, safety, events, poses = [], [], [], []

    def goal_cb(m):
        if st["goal"] is None:
            st.update(goal=m, t0=time.time())
            print("GOAL (%.2f, %.2f) yaw %.0f deg" % (m.pose.position.x, m.pose.position.y,
                                                      np.degrees(yaw_of(m.pose.orientation))), flush=True)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb)
    rospy.Subscriber("/move_base/current_goal", PoseStamped, goal_cb)
    rospy.Subscriber("/cmd_vel", Twist, lambda m: cmd.append((time.time(), m.linear.x, m.angular.z)) if st["goal"] else None)
    rospy.Subscriber("/scout_safety/state", String, lambda m: safety.append(json.loads(m.data)["state"]) if st["goal"] else None)
    rospy.Subscriber("/rosout_agg", Log, lambda m: events.append(m.msg[:90])
                     if st["goal"] and m.name == "/move_base" and m.level >= Log.WARN else None)

    def stat_cb(m):
        if st["goal"] and m.status_list:
            st["status"] = (m.status_list[-1].status, m.status_list[-1].text)
    rospy.Subscriber("/move_base/status", GoalStatusArray, stat_cb)

    end = time.time() + a.timeout
    while not rospy.is_shutdown() and time.time() < end:
        rospy.sleep(0.1)
        if not st["goal"]:
            continue
        try:
            t = buf.lookup_transform("map", "base_link", rospy.Time(0)).transform
            poses.append((time.time(), t.translation.x, t.translation.y, yaw_of(t.rotation)))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        if st["status"] and st["status"][0] in (2, 3, 4, 5, 8) and time.time() - st["t0"] > 2:
            break
    g = st["goal"]
    if g is None or not poses:
        print("no goal (or no pose) within %.0f s" % a.timeout)
        return
    gx, gy, gyaw = g.pose.position.x, g.pose.position.y, yaw_of(g.pose.orientation)
    P = np.array(poses)
    C = np.array(cmd) if cmd else np.zeros((0, 3))
    names = {2: "PREEMPTED", 3: "SUCCEEDED", 4: "ABORTED", 5: "REJECTED", 8: "RECALLED"}
    s = st["status"] or (None, "")
    print("result: %s %r after %.1f s" % (names.get(s[0], s[0]), s[1], time.time() - st["t0"]))
    d = np.hypot(P[:, 1] - gx, P[:, 2] - gy)
    near = int(np.argmax(d < 0.3)) if (d < 0.3).any() else None
    tnear = P[near, 0] if near is not None else np.inf
    drive, final = C[C[:, 0] < tnear], C[C[:, 0] >= tnear]
    moving = drive[drive[:, 1] > 0.02, 1]
    print("drive : %.1f s, %.2f m, mean speed while moving %.2f m/s, turn reversals %d" % (
        (tnear if near is not None else P[-1, 0]) - P[0, 0], np.linalg.norm(np.diff(P[:, 1:3], axis=0), axis=1).sum(),
        moving.mean() if len(moving) else 0.0, flips(drive[:, 2])))
    if near is not None:
        print("final : %.1f s within 0.3 m, %.1f s turning, turn reversals %d, total turned %.0f deg" % (
            P[-1, 0] - tnear, (np.abs(final[:, 2]) > 0.05).sum() / 50.0, flips(final[:, 2]),
            np.degrees(np.abs(np.diff(np.unwrap(P[near:, 3]))).sum())))
    dyaw = np.arctan2(np.sin(P[-1, 3] - gyaw), np.cos(P[-1, 3] - gyaw))
    print("end   : position error %.2f m, heading error %.0f deg" % (d[-1], abs(np.degrees(dyaw))))
    print("safety:", dict(Counter(safety)))
    ev = Counter(events)
    print("move_base warnings:", dict(ev.most_common(6)) if ev else "none")


if __name__ == "__main__":
    main()
