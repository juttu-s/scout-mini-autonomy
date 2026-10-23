#!/usr/bin/env python3
"""2D navigation grid by ray tracing (free-space carving) the lidar scans of a recorded session.

    rosrun scout_nav bag_to_raytraced_grid.py ~/maps/<name> [--out grid_rt]

The hit-count dense map only ever adds obstacles, so desks passed several times with slightly different DLIO drift,
and chairs that moved, end up fatter than reality: the planner then sees the robot's own cell as touching a desk
("NO PATH!", lab2 2026-10-19). Here every MID360 return in the obstacle height band is a ray from the lidar:
voxels it passes through get a "free" vote, the end voxel an "occupied" vote (log-odds, clamped), so later passes erase
obstacles stamped at drifted positions. Traced in 3D (10 cm layers across the band): a first 2D version carved desk
tops away, because rays ending higher in the band pass over them (only 58 % of live obstacles left on the map).
A cell is occupied if any voxel in its column is.

Kept from bag_to_dense_map.py: DLIO poses (mid-scan), returns < 1.2 m ignored (people next to the robot), the swept
volume (+-0.35 m of the driven path) forced free. Unknown = never crossed by a ray.
Writes <dir>/<out>.pgm + <out>.yaml (same origin/size as grid.yaml so they can be compared).
"""
import argparse
import glob
import os
import sys

import numpy as np
import rosbag

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from bag_to_dense_map import ODOM, LIDAR, quat_to_rot, slerp, xyz_from_cloud  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir")
    ap.add_argument("--out", default="grid_rt")
    ap.add_argument("--lidar-height", type=float, default=0.76)
    ap.add_argument("--band", type=float, nargs=2, default=[0.08, 0.95], help="obstacle band above the floor (m)")
    ap.add_argument("--min-range", type=float, default=1.2)
    ap.add_argument("--max-range", type=float, default=12.0)
    ap.add_argument("--path-clear", type=float, default=0.35)
    ap.add_argument("--every", type=int, default=2, help="use every Nth scan")
    ap.add_argument("--rays", type=int, default=3000, help="max rays per scan (random subset)")
    ap.add_argument("--hit", type=float, default=0.85)
    ap.add_argument("--miss", type=float, default=-0.4)
    ap.add_argument("--occ", type=float, default=1.2, help="log-odds above which a voxel is occupied")
    ap.add_argument("--dz", type=float, default=0.10, help="voxel layer height (m)")
    a = ap.parse_args()
    d = os.path.expanduser(a.session_dir)
    y = open(os.path.join(d, "grid.yaml")).read()
    res = float(y.split("resolution:")[1].split()[0])
    ox, oy = [float(v) for v in y.split("origin: [")[1].split("]")[0].split(",")[:2]]
    raw = open(os.path.join(d, "grid.pgm"), "rb").read().split(b"\n", 3)
    W, H = map(int, raw[1].split())
    bags = sorted(glob.glob(os.path.join(d, "recording", "*.bag")),
                  key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]))

    t, P, Q = [], [], []
    for b in bags:
        with rosbag.Bag(b) as bag:
            for _, m, _ in bag.read_messages(topics=[ODOM]):
                p, q = m.pose.pose.position, m.pose.pose.orientation
                t.append(m.header.stamp.to_sec()); P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
    o = np.argsort(t)
    t, P, Q = np.array(t)[o], np.array(P)[o], np.array(Q)[o]

    lo = a.band[0] - a.lidar_height  # band in the map frame (z = 0 at the lidar)
    hi = a.band[1] - a.lidar_height
    NZ = int(np.ceil((hi - lo) / a.dz))
    L = np.zeros(NZ * H * W, np.float32)  # log-odds voxels, index = (layer * H + row) * W + col
    def vidx(x, yy, z):
        k = ((z - lo) / a.dz).astype(np.int64); r_ = ((yy - oy) / res).astype(np.int64); c_ = ((x - ox) / res).astype(np.int64)
        ok = (k >= 0) & (k < NZ) & (r_ >= 0) & (r_ < H) & (c_ >= 0) & (c_ < W)
        return ((k * H + r_) * W + c_)[ok]
    rng = np.random.default_rng(0)
    steps = np.arange(0.0, a.max_range, res * 0.5)  # sample along rays at half a cell
    n = 0
    for b in bags:
        with rosbag.Bag(b) as bag:
            for k, (_, m, _) in enumerate(bag.read_messages(topics=[LIDAR])):
                if k % a.every:
                    continue
                ts = m.header.stamp.to_sec() + 0.05
                i = np.searchsorted(t, ts)
                if i == 0 or i >= len(t) or t[i] - t[i - 1] > 0.1:
                    continue
                f = (ts - t[i - 1]) / (t[i] - t[i - 1])
                pos = P[i - 1] + f * (P[i] - P[i - 1])
                R = quat_to_rot(slerp(Q[i - 1], Q[i], f))
                p = xyz_from_cloud(m)
                r = np.linalg.norm(p, axis=1)
                p = p[(r > a.min_range) & (r < a.max_range)]
                w = p @ R.T + pos
                w = w[(w[:, 2] > lo) & (w[:, 2] < hi)]
                if len(w) > a.rays:
                    w = w[rng.choice(len(w), a.rays, replace=False)]
                if not len(w):
                    continue
                v = w - pos
                dist = np.linalg.norm(v, axis=1)
                u = v / dist[:, None]
                # free samples along each 3D ray, stopping one cell short of the hit
                s = steps[None, :]
                keep = s < (dist[:, None] - res)
                ci = np.unique(vidx((pos[0] + u[:, 0:1] * s)[keep], (pos[1] + u[:, 1:2] * s)[keep],
                                    (pos[2] + u[:, 2:3] * s)[keep]))
                L[ci] += a.miss
                hi_c = np.unique(vidx(w[:, 0], w[:, 1], w[:, 2]))
                L[hi_c] += a.hit - a.miss * np.isin(hi_c, ci)  # a hit voxel crossed by another ray keeps its hit
                np.clip(L, -4.0, 4.0, out=L)
                n += 1
                if n % 200 == 0:
                    print("\r  %d scans" % n, end="", flush=True)
    print("\r  %d scans traced" % n)

    V = L.reshape(NZ, H, W)
    occ = (V > a.occ).any(axis=0)
    free = (V < -0.4).any(axis=0) & ~occ
    # swept volume: the robot was there
    import cv2
    pij = np.floor((P[:, :2] - [ox, oy]) / res).astype(int)
    mask = np.zeros((H, W), np.uint8)
    ok = (pij[:, 0] >= 0) & (pij[:, 1] >= 0) & (pij[:, 0] < W) & (pij[:, 1] < H)
    mask[pij[ok, 1], pij[ok, 0]] = 1
    r = int(round(a.path_clear / res))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))) > 0
    occ &= ~mask
    free |= mask
    img = np.full((H, W), 205, np.uint8)
    img[free] = 254
    img[occ] = 0
    with open(os.path.join(d, a.out + ".pgm"), "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (W, H))
        fh.write(np.flipud(img).tobytes())
    with open(os.path.join(d, a.out + ".yaml"), "w") as fh:
        fh.write("image: %s.pgm\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n"
                 "free_thresh: 0.196\n" % (a.out, res, ox, oy))
    print("%s: %d occupied, %d free, %d unknown cells" % (a.out, occ.sum(), free.sum(), (img == 205).sum()))


if __name__ == "__main__":
    main()
