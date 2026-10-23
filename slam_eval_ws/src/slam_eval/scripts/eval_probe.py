#!/usr/bin/env python3
"""Per-scan latency and trajectory recorder for one SLAM method (started by run_slam_eval.py).

    rosrun slam_eval eval_probe.py --input T --output T --odom T --out DIR --name NAME
                                   [--stamp-offset S] [--body-to-lidar X Y Z] [--final-path T]

Latency = wall time the method's per-scan output arrives - wall time its input arrived, both seen by this process
(same clock, so the bag player's timing doesn't matter). Outputs are matched to the input whose stamp is nearest to
output stamp - stamp_offset, within 30 ms (MID360 scans are 100 ms apart). Messages are only read as far as their
header (rospy.AnyMsg), so the probe costs almost nothing and works for any message type.
On shutdown writes <name>_online.tum (odometry, in the MID360 frame), <name>_final.tum (if --final-path),
<name>_latency.csv and <name>_probe.json (counts).
"""
import argparse
import bisect
import json
import os
import struct
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path

MATCH_TOL = 0.03


def stamp_of(raw):
    """Header stamp from the serialized bytes (std_msgs/Header is first: seq, secs, nsecs)."""
    _, secs, nsecs = struct.unpack_from("<III", raw._buff, 0)
    return secs + nsecs * 1e-9


def rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def write_tum(path, rows):
    with open(path, "w") as f:
        for r in rows:
            f.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % tuple(r))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--stamp-offset", type=float, default=0.0)
    ap.add_argument("--body-to-lidar", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    ap.add_argument("--final-path")
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("eval_probe")
    off = np.array(a.body_to_lidar)
    lock = threading.Lock()
    ins, outs, traj, final, n_lost = {}, [], [], {"path": None}, [0]

    def in_cb(m):
        t = time.time()
        with lock:
            ins.setdefault(stamp_of(m), t)

    def out_cb(m):
        t = time.time()
        with lock:
            outs.append((stamp_of(m), t))

    def odom_cb(m):
        if m.header.stamp.is_zero():  # DLIO before its IMU calibration
            return
        if m.pose.covariance[0] >= 9999:  # RTAB-Map publishes a null pose with huge covariance when lost
            n_lost[0] += 1
            return
        p, q = m.pose.pose.position, m.pose.pose.orientation
        qq = (q.x, q.y, q.z, q.w)
        pos = np.array([p.x, p.y, p.z]) + rotate(qq, off)
        traj.append((m.header.stamp.to_sec(), *pos, *qq))

    def odom_out_cb(m):  # odometry is also the per-scan output
        t = time.time()
        with lock:
            outs.append((m.header.stamp.to_sec(), t))
        odom_cb(m)

    rospy.Subscriber(a.input, rospy.AnyMsg, in_cb, queue_size=200, tcp_nodelay=True)
    if a.output == a.odom:
        rospy.Subscriber(a.odom, Odometry, odom_out_cb, queue_size=200, tcp_nodelay=True)
    else:
        rospy.Subscriber(a.output, rospy.AnyMsg, out_cb, queue_size=200, tcp_nodelay=True)
        rospy.Subscriber(a.odom, Odometry, odom_cb, queue_size=500, tcp_nodelay=True)
    if a.final_path:
        rospy.Subscriber(a.final_path, Path, lambda m: final.update(path=m), queue_size=1)

    def dump():
        with lock:
            stamps = sorted(ins)
            o = list(outs)
        lat, used = [], set()
        for s, t in o:
            target = s - a.stamp_offset
            i = bisect.bisect_left(stamps, target)
            best = min((j for j in (i - 1, i) if 0 <= j < len(stamps)), key=lambda j: abs(stamps[j] - target), default=None)
            if best is None or abs(stamps[best] - target) > MATCH_TOL or best in used:
                continue
            used.add(best)
            lat.append((stamps[best], s, ins[stamps[best]], t, t - ins[stamps[best]]))
        pre = os.path.join(a.out, a.name)
        with open(pre + "_latency.csv", "w") as f:
            f.write("input_stamp,output_stamp,input_wall,output_wall,latency_s\n")
            f.writelines("%.6f,%.6f,%.6f,%.6f,%.6f\n" % r for r in lat)
        write_tum(pre + "_online.tum", sorted(traj))
        n_final = 0
        if final["path"] is not None:
            rows = []
            for ps in final["path"].poses:
                p, q = ps.pose.position, ps.pose.orientation
                qq = (q.x, q.y, q.z, q.w)
                rows.append((ps.header.stamp.to_sec(), *(np.array([p.x, p.y, p.z]) + rotate(qq, off)), *qq))
            write_tum(pre + "_final.tum", rows)
            n_final = len(rows)
        with open(pre + "_probe.json", "w") as f:
            first = lat[0][0] if lat else float("inf")  # scans before the first output: initialisation
            json.dump({"inputs": len(stamps), "inputs_after_init": sum(1 for x in stamps if x >= first),
                       "outputs": len(o), "matched": len(lat), "odom_poses": len(traj),
                       "odom_lost": n_lost[0], "final_poses": n_final}, f, indent=1)

    rospy.on_shutdown(dump)
    rospy.spin()


if __name__ == "__main__":
    main()
