#!/usr/bin/env python3
"""Comparison table and plots for a run_slam_eval.py results directory (evo).

    ~/ros_utils/slam_eval_ws/venv/bin/python compare_slam_eval.py <results dir> [--reference lio_sam:final]
                                                                  [--rpe-delta 10] [--max-gap 5]

(evo lives in that venv so its newer numpy can't disturb the robot's Python.)
There is no ground truth, so accuracy is reported two ways:
  end-point drift  the session starts and ends at the same mark, so |p_end - p_start| (and the yaw gap) is each
                   method's accumulated error; "online" is the odometry as it ran, "final" the loop-closed trajectory
  agreement        APE / RPE against a reference method (interpolated at each method's stamps, SE3 Umeyama
                   alignment, no scale): how much two methods
                   agree, not which is right
plus the cost numbers from metrics.json (mean +- sd over repeats).
Writes comparison.md, comparison.csv, trajectories_xy.png, trajectories_z.png, latency.png into the results dir.
"""
import argparse
import copy
import glob
import json
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from evo.core import lie_algebra, metrics  # noqa: E402
from evo.core.trajectory import PoseTrajectory3D  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402
from evo.core.geometry import GeometryException  # noqa: E402
from evo.core.metrics import PoseRelation, StatisticsType, Unit  # noqa: E402
from evo.tools import file_interface  # noqa: E402

ORDER = ["dlio", "fast_lio", "lio_sam", "rtabmap_icp", "rtabmap_rgbd"]


def yaw(q_wxyz):
    w, x, y, z = q_wxyz
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_runs(d):
    """{method: [run dir, ...]} (repeats are <method>_rN)."""
    runs = {}
    for m in sorted(glob.glob(os.path.join(d, "*", "metrics.json"))):
        sub = os.path.basename(os.path.dirname(m))
        runs.setdefault(re.sub(r"_r\d+$", "", sub), []).append(os.path.dirname(m))
    return dict(sorted(runs.items(), key=lambda kv: ORDER.index(kv[0]) if kv[0] in ORDER else 99))


def read_tum(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    t = file_interface.read_tum_trajectory_file(path)
    return t if t.num_poses >= 2 else None


def path_length(p, step=0.05):
    """Driven distance, counting a move only once the pose is `step` m from the last counted one (otherwise pose
    jitter while standing still adds length: FAST-LIO at 100 Hz vs DLIO differed by 8 % on the same drive)."""
    n, last = 0.0, p[0]
    for x in p[1:]:
        d = np.linalg.norm(x - last)
        if d >= step:
            n, last = n + d, x
    return n


def endpoint(t):
    p = t.positions_xyz
    q = t.orientations_quat_wxyz
    dyaw = np.degrees(np.arctan2(np.sin(yaw(q[-1]) - yaw(q[0])), np.cos(yaw(q[-1]) - yaw(q[0]))))
    gap = float(np.linalg.norm(p[-1] - p[0]))
    length = path_length(p)
    return gap, 100.0 * gap / max(length, 1e-6), abs(dyaw), length


def ref_at(ref, stamps, max_gap):
    """Reference poses interpolated at `stamps` (position linear, rotation slerp) where the reference has poses at
    most `max_gap` s apart on both sides (None if there are none). A loop-closed reference has one pose per keyframe (LIO-SAM: ~1 m apart),
    so nearest-stamp association alone matched only a handful of another method's poses."""
    t = ref.timestamps
    i = np.searchsorted(t, stamps)
    ok = (i > 0) & (i < len(t))
    ok[ok] &= (t[i[ok]] - t[i[ok] - 1]) <= max_gap
    exact = np.isin(stamps, t)
    ok |= exact
    s = stamps[ok]
    if not len(s):  # no overlap at all (Slerp can't evaluate an empty set)
        return None, ok
    pos = np.stack([np.interp(s, t, ref.positions_xyz[:, k]) for k in range(3)], 1)
    rot = Slerp(t, Rotation.from_quat(ref.orientations_quat_wxyz[:, [1, 2, 3, 0]]))(s).as_quat()[:, [3, 0, 1, 2]]
    return PoseTrajectory3D(positions_xyz=pos, orientations_quat_wxyz=rot, timestamps=s), ok


def agreement(ref, est, a):
    """(APE rmse m, RPE rmse m per delta, full est aligned to ref) or Nones when they don't overlap."""
    r, ok = ref_at(ref, est.timestamps, a.max_gap)
    if r is None or r.num_poses < 10:
        return None, None, None
    e = PoseTrajectory3D(positions_xyz=est.positions_xyz[ok], orientations_quat_wxyz=est.orientations_quat_wxyz[ok],
                         timestamps=est.timestamps[ok])
    try:
        rot, trans, _ = e.align(r, correct_scale=False)
    except GeometryException:  # matched poses all in one spot (e.g. a method that never moved)
        return None, None, None
    ape = metrics.APE(PoseRelation.translation_part)
    ape.process_data((r, e))
    rpe_v = None
    try:
        rpe = metrics.RPE(PoseRelation.translation_part, delta=a.rpe_delta, delta_unit=Unit.meters, all_pairs=False)
        rpe.process_data((r, e))
        rpe_v = rpe.get_statistic(StatisticsType.rmse)
    except metrics.MetricsException:
        pass
    full = copy.deepcopy(est)  # the whole trajectory, with the alignment found on the overlapping part (for plots)
    full.transform(lie_algebra.se3(rot, trans))
    return ape.get_statistic(StatisticsType.rmse), rpe_v, full


def mean_sd(vals, fmt="%.1f"):
    v = [x for x in vals if x is not None]
    if not v:
        return "-"
    return fmt % np.mean(v) if len(v) == 1 else (fmt + " ± " + fmt) % (np.mean(v), np.std(v))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results_dir")
    ap.add_argument("--reference", default="lio_sam:final", help="method:online|final used for APE/RPE")
    ap.add_argument("--rpe-delta", type=float, default=10.0, help="m")
    ap.add_argument("--max-gap", type=float, default=5.0, help="s, longest reference gap to interpolate across")
    a = ap.parse_args()
    d = os.path.expanduser(a.results_dir)
    runs = load_runs(d)
    if not runs:
        raise SystemExit("no */metrics.json in %s" % d)

    # trajectories of the first repeat of each method (repeats only feed the cost statistics)
    traj = {}
    for m, dirs in runs.items():
        for kind in ("online", "final"):
            t = read_tum(os.path.join(dirs[0], "%s_%s.tum" % (m, kind)))
            if t is not None:
                traj[(m, kind)] = t
    rm, rk = a.reference.split(":") if ":" in a.reference else (a.reference, "final")
    if not traj:
        raise SystemExit("no trajectories in %s" % d)
    cands = [(rm, rk), (rm, "online")] + sorted(traj, key=lambda k: k[1] != "final")  # else any loop-closed one
    ref_key = next(k for k in cands if k in traj)
    if ref_key != (rm, rk):
        print("reference %s:%s not available, using %s:%s" % (rm, rk, *ref_key))
    ref = traj[ref_key]

    rows, aligned = [], {}
    for m, dirs in runs.items():
        ms = [json.load(open(os.path.join(r, "metrics.json"))) for r in dirs]
        cost = {
            "status": ", ".join(sorted({x["status"] for x in ms})),
            "cpu_s": mean_sd([x.get("cpu_s") for x in ms], "%.0f"),
            "cores": mean_sd([x.get("mean_cores") for x in ms], "%.2f"),
            "rss": mean_sd([x.get("peak_rss_mb") for x in ms], "%.0f"),
            "p50": mean_sd([x.get("latency_ms", {}).get("p50") for x in ms]),
            "p95": mean_sd([x.get("latency_ms", {}).get("p95") for x in ms]),
            "max": mean_sd([x.get("latency_ms", {}).get("max") for x in ms], "%.0f"),
            "drop": mean_sd([x.get("dropped_after_init_pct") for x in ms]),
            "wall": mean_sd([x.get("processing_wall_s") for x in ms], "%.0f"),
        }
        for kind in ("online", "final"):
            t = traj.get((m, kind))
            if t is None:
                if kind == "online":
                    rows.append(dict(method=m, traj=kind, **cost))
                continue
            gap, gap_pct, dyaw, length = endpoint(t)
            ape, rpe, e = (None, None, None) if (m, kind) == ref_key else agreement(ref, t, a)
            if e is not None:
                aligned[(m, kind)] = e
            rows.append(dict(method=m, traj=kind, length="%.1f" % length, end_gap="%.2f" % gap, end_pct="%.2f" % gap_pct,
                             end_yaw="%.1f" % dyaw, ape="ref" if (m, kind) == ref_key else ("%.3f" % ape if ape is not None else "-"),
                             rpe="ref" if (m, kind) == ref_key else ("%.2f" % (100 * rpe / a.rpe_delta) if rpe is not None else "-"),
                             **cost))

    cols = [("method", "method"), ("traj", "trajectory"), ("status", "status"), ("length", "path m"),
            ("end_gap", "end gap m"), ("end_pct", "end gap %"), ("end_yaw", "end yaw °"),
            ("ape", "APE rmse m*"), ("rpe", "RPE %%/%gm*" % a.rpe_delta), ("p50", "latency p50 ms"),
            ("p95", "p95 ms"), ("max", "max ms"), ("drop", "dropped % after init"), ("cpu_s", "CPU s"), ("cores", "mean cores"),
            ("rss", "peak RSS MB"), ("wall", "wall s")]
    # repeat the cost columns only on the first row of a method
    seen = set()
    table = []
    for r in rows:
        r = {k: r.get(k, "-") for k, _ in cols}
        if r["method"] in seen:
            for k in ("status", "p50", "p95", "max", "drop", "cpu_s", "cores", "rss", "wall"):
                r[k] = ""
        seen.add(r["method"])
        table.append(r)
    meta = json.load(open(os.path.join(d, "meta.json"))) if os.path.exists(os.path.join(d, "meta.json")) else {}
    with open(os.path.join(d, "comparison.md"), "w") as f:
        f.write("# SLAM comparison: %s\n\n" % os.path.basename(meta.get("session", d)))
        f.write("Rate %s×, repeats %s, %s. Reference for APE/RPE: **%s:%s**.\n\n" % (
            meta.get("rate", "?"), meta.get("repeats", "?"), meta.get("nvpmodel", "").splitlines()[0] if meta.get("nvpmodel") else "",
            *ref_key))
        f.write("| " + " | ".join(h for _, h in cols) + " |\n|" + "---|" * len(cols) + "\n")
        for r in table:
            f.write("| " + " | ".join(str(r[k]) for k, _ in cols) + " |\n")
        f.write("\n\\* agreement with the reference after SE3 alignment, not accuracy (no ground truth). "
                "End gap: the session starts and ends at the same mark, so this is the accumulated drift "
                "(online = odometry as it ran, final = after loop closure). Latency: input received → per-scan output "
                "received. CPU s: user+system time of the method's processes while the bag played.\n")
    with open(os.path.join(d, "comparison.csv"), "w") as f:
        f.write(",".join(k for k, _ in cols) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(k, "")).replace(",", ";") for k, _ in cols) + "\n")

    # plots: every trajectory aligned to the reference
    names = sorted({k for k in traj}, key=lambda k: (ORDER.index(k[0]) if k[0] in ORDER else 99, k[1]))
    colors = {m: c for m, c in zip(ORDER, plt.rcParams["axes.prop_cycle"].by_key()["color"])}
    fig, ax = plt.subplots(figsize=(9, 9))
    figz, axz = plt.subplots(figsize=(11, 4))
    t0 = ref.timestamps[0]
    for k in names:
        t = ref if k == ref_key else aligned.get(k)
        if t is None:
            continue
        p = t.positions_xyz
        ls = "-" if k[1] == "final" or (k[0], "final") not in traj else ":"
        lw = 2.2 if k == ref_key else 1.2
        lab = "%s %s%s" % (k[0], k[1], " (ref)" if k == ref_key else "")
        ax.plot(p[:, 0], p[:, 1], ls, color=colors.get(k[0], "k"), lw=lw, label=lab)
        ax.plot(p[-1, 0], p[-1, 1], "x", color=colors.get(k[0], "k"), ms=8)
        axz.plot(t.timestamps - t0, p[:, 2], ls, color=colors.get(k[0], "k"), lw=lw, label=lab)
    ax.plot(*ref.positions_xyz[0, :2], "ko", ms=8, label="start")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("Trajectories aligned to %s:%s (x = end)" % ref_key)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "trajectories_xy.png"), dpi=150)
    axz.set_xlabel("time (s)")
    axz.set_ylabel("z (m)")
    axz.grid(alpha=0.3)
    axz.legend(fontsize=8, ncol=3)
    axz.set_title("Height (the floor is flat: any change is vertical drift)")
    figz.tight_layout()
    figz.savefig(os.path.join(d, "trajectories_z.png"), dpi=150)

    lat, labels = [], []
    for m, dirs in runs.items():
        v = []
        for r in dirs:
            f = os.path.join(r, m + "_latency.csv")
            if os.path.exists(f):
                x = np.loadtxt(f, delimiter=",", skiprows=1, ndmin=2)
                if len(x):
                    v.append(x[:, 4] * 1000)
        if v:
            lat.append(np.concatenate(v))
            labels.append(m)
    if lat:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.boxplot(lat, labels=labels, showfliers=False, whis=(5, 95))
        ax.set_ylabel("per-scan latency (ms), whiskers 5-95 %")
        ax.grid(axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(d, "latency.png"), dpi=150)
    print(open(os.path.join(d, "comparison.md")).read())


if __name__ == "__main__":
    main()
