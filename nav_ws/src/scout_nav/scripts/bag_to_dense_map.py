#!/usr/bin/env python3
"""Build a dense 3D map offline from a recorded session (record_session.sh), then the navigation grid.

    rosrun scout_nav bag_to_dense_map.py ~/maps/<name> [--voxel 0.05] [--min-hits 3]

DLIO's saved map is sparse: its keyframes are voxelized at 25 cm by DLIO's input preprocessing (needed for its
odometry), so walls come out as dotted lines. The bag has every full-resolution MID360 scan and DLIO's 100 Hz pose;
this places each scan at the pose for the middle of the scan, merges them into voxels, and keeps voxels hit at
least --min-hits times (removes people walking by and stray returns; walls are hit hundreds of times).
Returns within --min-range (1.2 m) of the lidar are ignored: someone following the robot during mapping is always
that close, and was otherwise stamped as a wall along the whole driven path (lab2: 51 % of the path blocked).
The space the robot drove through is known to be empty up to robot height, so voxels within --path-clear (0.35 m)
of the trajectory and below the robot's top are removed too (people following further back walk on that path).

Writes <dir>/dlio_map.pcd (dense; the original is kept as dlio_map_sparse.pcd) and rebuilds the grid with
pcd_to_grid.py, so localization and navigation use it unchanged. Scans are not deskewed (fine at teleop speed).
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

import numpy as np
import rosbag

LIDAR, ODOM = "/livox/lidar", "/dlio/odom_node/odom"


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp(q0, q1, f):
    d = np.dot(q0, q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + f * (q1 - q0)
    else:
        th = np.arccos(d)
        q = (np.sin((1 - f) * th) * q0 + np.sin(f * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


def xyz_from_cloud(msg):
    offs = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [offs["x"], offs["y"], offs["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)


def lidar_extrinsic(bags):
    """(R, t) of livox_frame in base_link from the recording's /tf_static, identity if it isn't there.

    Older sessions (e.g. lab2, 2026-10-18) used the lidar itself as base_link and recorded no such transform. Since
    2026-10-23 base_link is the turning centre and robot.launch publishes base_link -> livox_frame (0.178 m ahead), so
    DLIO's poses are the centre's and every scan must be shifted by this before placing it in the map.
    """
    for b in bags[:1]:
        with rosbag.Bag(b) as bag:
            for _, m, _ in bag.read_messages(topics=["/tf_static"]):
                for tr in m.transforms:
                    if tr.header.frame_id.lstrip("/") == "base_link" and tr.child_frame_id.lstrip("/") == "livox_frame":
                        q, v = tr.transform.rotation, tr.transform.translation
                        return quat_to_rot((q.x, q.y, q.z, q.w)), np.array([v.x, v.y, v.z])
    return np.eye(3), np.zeros(3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir")
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--min-hits", type=int, default=3)
    ap.add_argument("--min-range", type=float, default=1.2,
                    help="ignore returns closer than this to the lidar: the Livox (0,0,0) no-return points, the robot "
                         "itself, and whoever walks next to / behind the robot while driving it (they would otherwise "
                         "be stamped along the whole path). Structure beside the path is still seen from further away")
    ap.add_argument("--max-range", type=float, default=25.0)
    ap.add_argument("--path-clear", type=float, default=0.35,
                    help="m either side of the driven path cleared up to robot height (half-width 0.29 + margin)")
    ap.add_argument("--lidar-height", default="0.76")
    ap.add_argument("--robot-height", default="0.85")
    a = ap.parse_args()
    d = os.path.expanduser(a.session_dir)
    bags = sorted(glob.glob(os.path.join(d, "recording", "*.bag")),
                  key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]) if p.rsplit("_", 1)[1].split(".")[0].isdigit() else 0)
    if not bags:
        sys.exit("no bags in %s/recording" % d)

    Rl, tl = lidar_extrinsic(bags)
    print("lidar in base_link: t = %s%s" % (np.round(tl, 3), "" if tl.any() else " (base_link = lidar)"))

    # pass 1: DLIO poses (base_link, in DLIO's odom frame = the saved map frame)
    t, P, Q = [], [], []
    for b in bags:
        with rosbag.Bag(b) as bag:
            for _, m, _ in bag.read_messages(topics=[ODOM]):
                p, q = m.pose.pose.position, m.pose.pose.orientation
                t.append(m.header.stamp.to_sec()); P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
    t, P, Q = np.array(t), np.array(P), np.array(Q)
    order = np.argsort(t)
    t, P, Q = t[order], P[order], Q[order]
    print("%d DLIO poses, %.0f s, path %.1f m" % (len(t), t[-1] - t[0], np.linalg.norm(np.diff(P, axis=0), axis=1).sum()))

    # pass 2: scans -> map frame -> voxel hit counts
    keys_all, counts_all, n_scans, n_pts, skipped = None, None, 0, 0, 0
    pending = []
    off = 1 << 20

    def merge(pending, keys_all, counts_all):
        k = np.concatenate(pending)
        k, c = np.unique(k, return_counts=True)
        if keys_all is not None:
            k = np.concatenate([keys_all, k]); c = np.concatenate([counts_all, c])
            k, inv = np.unique(k, return_inverse=True)
            c = np.bincount(inv, weights=c).astype(np.int64)
        return k, c

    for b in bags:
        with rosbag.Bag(b) as bag:
            for _, m, _ in bag.read_messages(topics=[LIDAR]):
                ts = m.header.stamp.to_sec() + 0.05  # middle of the 100 ms MID360 scan
                i = np.searchsorted(t, ts)
                if i == 0 or i >= len(t) or t[i] - t[i - 1] > 0.1:
                    skipped += 1
                    continue
                f = (ts - t[i - 1]) / (t[i] - t[i - 1])
                pos = P[i - 1] + f * (P[i] - P[i - 1])
                R = quat_to_rot(slerp(Q[i - 1], Q[i], f))
                p = xyz_from_cloud(m)
                r = np.linalg.norm(p, axis=1)
                p = p[(r > a.min_range) & (r < a.max_range)]
                w = (p @ Rl.T + tl) @ R.T + pos
                k = np.floor(w / a.voxel).astype(np.int64) + off
                pending.append((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2])
                n_scans += 1
                n_pts += len(p)
                if len(pending) >= 200:
                    keys_all, counts_all = merge(pending, keys_all, counts_all)
                    pending = []
                    print("\r  %d scans, %d voxels" % (n_scans, len(keys_all)), end="", flush=True)
    if pending:
        keys_all, counts_all = merge(pending, keys_all, counts_all)
    print("\r  %d scans (%d skipped without a pose), %.1f M points -> %d voxels" % (n_scans, skipped, n_pts / 1e6, len(keys_all)))

    keep = keys_all[counts_all >= a.min_hits]
    mask = (1 << 21) - 1
    ijk = np.stack([(keep >> 42) & mask, (keep >> 21) & mask, keep & mask], 1) - off
    xyz = ((ijk + 0.5) * a.voxel).astype(np.float32)
    print("  kept %d voxels hit >= %d times" % (len(xyz), a.min_hits))

    # Swept volume: the robot occupied these columns, so nothing below its top can be static there
    import cv2
    res = 0.05
    mn = P[:, :2].min(0) - a.path_clear - 0.1
    pij = np.floor((P[:, :2] - mn) / res).astype(int)
    W, H = pij.max(0) + int(np.ceil(a.path_clear / res)) + 3
    mask = np.zeros((H, W), np.uint8)
    mask[pij[:, 1], pij[:, 0]] = 1
    r = int(round(a.path_clear / res))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
    q = np.floor((xyz[:, :2] - mn) / res).astype(int)
    inside = (q[:, 0] >= 0) & (q[:, 1] >= 0) & (q[:, 0] < W) & (q[:, 1] < H)
    swept = np.zeros(len(xyz), bool)
    swept[inside] = mask[q[inside, 1], q[inside, 0]] > 0
    top = -float(a.lidar_height) + float(a.robot_height) + 0.1  # robot top (+ margin) in the map frame
    drop = swept & (xyz[:, 2] < top)
    xyz = xyz[~drop]
    print("  removed %d voxels inside the robot's swept volume (people following it); %d left" % (drop.sum(), len(xyz)))

    sparse = os.path.join(d, "dlio_map_sparse.pcd")
    if not os.path.exists(sparse) and os.path.exists(os.path.join(d, "dlio_map.pcd")):
        shutil.copy2(os.path.join(d, "dlio_map.pcd"), sparse)
    with open(os.path.join(d, "dlio_map.pcd"), "wb") as fh:
        fh.write(("# .PCD v0.7 - dense map from %s (voxel %.2f m, >= %d hits)\nVERSION 0.7\nFIELDS x y z\n"
                  "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\n"
                  "DATA binary\n" % (os.path.basename(d), a.voxel, a.min_hits, len(xyz), len(xyz))).encode())
        fh.write(xyz.tobytes())
    print("wrote %s (original kept as dlio_map_sparse.pcd)" % os.path.join(d, "dlio_map.pcd"))
    subprocess.run(["rosrun", "scout_nav", "pcd_to_grid.py", d, "--lidar-height", a.lidar_height,
                    "--robot-height", a.robot_height], check=True)


if __name__ == "__main__":
    main()
