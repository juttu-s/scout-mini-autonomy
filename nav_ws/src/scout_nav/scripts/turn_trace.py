#!/usr/bin/env python3
"""Trace the next move_base goal and analyse its turns in place.

    rosrun scout_nav turn_trace.py [--out DIR]

Logs /nav/cmd_vel (move_base), /cmd_vel (after safety), /odom (wheels) and /localization/odom (base_link in map)
until the goal ends, writes them to <DIR>/turn_trace_<time>.csv (default ~/maps/turn_traces), and prints, per
turn in place longer than 1 s (wheels |v| < 0.03 m/s, |w| > 0.05 rad/s):
  - degrees turned, and how far base_link moved per radian: ~0 if base_link is the turning centre; an
    off-centre base_link circles (it was the lidar, 0.178 m ahead of the centre, until 2026-10-23)
  - the radius of a circle fitted to base_link's track
  - commanded (move_base / after safety) vs measured turn rate, and how often move_base flips direction (hunting)
  - heading error to the goal at the end of the turn
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="~/maps/turn_traces")
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("turn_trace", anonymous=True)
    st = {"goal": None, "done": None}
    nav, cmd, wheel, loc = [], [], [], []
    on = lambda: st["goal"] is not None
    rospy.Subscriber("/move_base/current_goal", PoseStamped, lambda m: st.update(goal=m) if st["goal"] is None else None)
    rospy.Subscriber("/nav/cmd_vel", Twist, lambda m: nav.append((time.time(), m.linear.x, m.angular.z)) if on() else None)
    rospy.Subscriber("/cmd_vel", Twist, lambda m: cmd.append((time.time(), m.linear.x, m.angular.z)) if on() else None)
    rospy.Subscriber("/odom", Odometry, lambda m: wheel.append(
        (time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z)) if on() else None)
    rospy.Subscriber("/localization/odom", Odometry, lambda m: loc.append(
        (time.time(), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))) if on() else None)

    def status(m):
        if on() and m.status_list and m.status_list[-1].status in (2, 3, 4, 5, 8) and st["done"] is None:
            st["done"] = (time.time(), m.status_list[-1].status)
    rospy.Subscriber("/move_base/status", GoalStatusArray, status)
    print("waiting for a goal ...", flush=True)
    while not rospy.is_shutdown() and (st["goal"] is None or st["done"] is None or time.time() < st["done"][0] + 1.0):
        time.sleep(0.1)
    g = st["goal"].pose
    gyaw = yaw_of(g.orientation)
    print("goal (%.2f, %.2f) yaw %.0f deg, status %d" % (g.position.x, g.position.y, math.degrees(gyaw), st["done"][1]))
    d = os.path.expanduser(a.out)
    os.makedirs(d, exist_ok=True)
    out = os.path.join(d, "turn_trace_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["topic", "t", "a", "b", "c"])
        for name, rows in (("nav", nav), ("cmd", cmd), ("wheel", wheel), ("loc", loc)):
            for r in rows:
                w.writerow([name] + list(r))
    print("trace:", out)
    W, C, N, L = (np.array(x) if x else np.zeros((0, 4)) for x in (wheel, cmd, nav, loc))
    if not len(W) or not len(L):
        print("no wheel odometry or localization recorded")
        return
    t0 = L[0, 0]
    turn = (np.abs(W[:, 1]) < 0.03) & (np.abs(W[:, 2]) > 0.05)
    segs, start = [], None
    for i, flag in enumerate(turn):
        if flag and start is None:
            start = i
        if (not flag or i == len(turn) - 1) and start is not None:
            if W[i - 1, 0] - W[start, 0] > 1.0:
                segs.append((W[start, 0], W[i - 1, 0]))
            start = None
    for t_a, t_b in segs:
        sel = lambda X: X[(X[:, 0] >= t_a) & (X[:, 0] <= t_b)] if len(X) else X
        P, Cs, Ns, Ws = sel(L), sel(C), sel(N), sel(W)
        if len(P) < 5:
            continue
        dyaw = np.abs(np.diff(np.unwrap(P[:, 3]))).sum()
        move = np.linalg.norm(np.diff(P[:, 1:3], axis=0), axis=1).sum()
        x, y = P[:, 1], P[:, 2]
        sol = np.linalg.lstsq(np.c_[2 * x, 2 * y, np.ones(len(x))], x ** 2 + y ** 2, rcond=None)[0]
        R = math.sqrt(max(sol[2] + sol[0] ** 2 + sol[1] ** 2, 0.0))
        nz = Ns[np.abs(Ns[:, 2]) > 0.02, 2] if len(Ns) else np.zeros(0)
        flips = int((np.diff(np.sign(nz)) != 0).sum()) if len(nz) > 1 else 0
        err = abs(math.degrees(math.atan2(math.sin(P[-1, 3] - gyaw), math.cos(P[-1, 3] - gyaw))))
        print("turn in place t=%.1f..%.1f s: turned %.0f deg, base_link moved %.3f m (%.3f m/rad), fitted radius %.3f m, "
              "|w| move_base %.2f / after safety %.2f / wheels %.2f rad/s, move_base direction flips %d, "
              "heading error at the end %.0f deg" % (
                  t_a - t0, t_b - t0, math.degrees(dyaw), move, move / max(dyaw, 1e-3), R,
                  np.abs(Ns[:, 2]).mean() if len(Ns) else 0, np.abs(Cs[:, 2]).mean() if len(Cs) else 0,
                  np.abs(Ws[:, 2]).mean(), flips, err))
    if not segs:
        print("no turn in place longer than 1 s")


if __name__ == "__main__":
    main()
