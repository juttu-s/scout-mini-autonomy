"""Unit tests for compare_slam_eval.py metrics on synthetic trajectories. Run: pytest slam_eval_ws/src/slam_eval/test
(needs the packages in ../requirements.txt: evo, scipy, matplotlib, numpy)."""
import argparse
import importlib
import os
import sys

import numpy as np
import pytest
from evo.core.trajectory import PoseTrajectory3D
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
cse = importlib.import_module("compare_slam_eval")


def traj(xyz, yaw_deg, t):
    q = Rotation.from_euler("z", yaw_deg, degrees=True).as_quat()[:, [3, 0, 1, 2]]  # wxyz
    return PoseTrajectory3D(positions_xyz=np.asarray(xyz, float), orientations_quat_wxyz=q,
                            timestamps=np.asarray(t, float))


def square_loop(n=400, side=5.0, end_gap=(0.0, 0.0), end_yaw=0.0):
    """A closed square driven at constant speed, the last pose offset by end_gap / end_yaw (drift)."""
    s = np.linspace(0, 4 * side, n)
    xy = np.empty((n, 2))
    for i, d in enumerate(s):
        k, r = divmod(d, side)
        xy[i] = [(r, 0), (side, r), (side - r, side), (0, side - r), (0, 0)][int(k)]
    xy[-1] += end_gap
    yaw = np.degrees(np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0])))
    yaw[0], yaw[-1] = 0.0, end_yaw
    return traj(np.c_[xy, np.zeros(n)], yaw, np.arange(n) * 0.1)


def test_path_length_straight_line():
    p = np.c_[np.linspace(0, 10, 1001), np.zeros(1001), np.zeros(1001)]
    assert cse.path_length(p) == pytest.approx(10.0, abs=0.05)


def test_path_length_ignores_standing_jitter():
    rng = np.random.default_rng(0)
    p = rng.normal(0, 0.01, (2000, 3))  # 1 cm noise while standing still
    assert cse.path_length(p) < 0.5
    assert np.linalg.norm(np.diff(p, axis=0), axis=1).sum() > 20  # what naive summing would report


def test_endpoint_closed_loop_drift():
    gap, pct, dyaw, length = cse.endpoint(square_loop(end_gap=(0.3, 0.4), end_yaw=10.0))
    assert gap == pytest.approx(0.5, abs=1e-6)
    assert length == pytest.approx(20.0, rel=0.03)
    assert pct == pytest.approx(100 * gap / length)
    assert dyaw == pytest.approx(10.0, abs=1e-6)


def test_ref_at_interpolates_and_skips_gaps():
    ref = traj([[0, 0, 0], [1, 0, 0], [2, 0, 0], [10, 0, 0]], [0, 0, 90, 90], [0.0, 1.0, 2.0, 10.0])
    r, ok = cse.ref_at(ref, np.array([-1.0, 0.5, 1.5, 2.0, 6.0, 11.0]), max_gap=2.0)
    assert ok.tolist() == [False, True, True, True, False, False]  # before, inside, inside, exact, gap, after
    assert r.positions_xyz[:, 0] == pytest.approx([0.5, 1.5, 2.0])
    yaws = Rotation.from_quat(r.orientations_quat_wxyz[:, [1, 2, 3, 0]]).as_euler("zyx", degrees=True)[:, 0]
    assert yaws == pytest.approx([0.0, 45.0, 90.0])


def test_agreement_identical_up_to_rigid_transform():
    ref = square_loop()
    rot = Rotation.from_euler("z", 30, degrees=True)
    est = traj(rot.apply(ref.positions_xyz) + [2.0, -1.0, 0.5],
               np.degrees(Rotation.from_quat(ref.orientations_quat_wxyz[:, [1, 2, 3, 0]]).as_euler("zyx")[:, 0]) + 30,
               ref.timestamps + 0.05)  # different stamps: interpolated reference
    a = argparse.Namespace(max_gap=1.0, rpe_delta=2.0)
    ape, rpe, full = cse.agreement(ref, est, a)
    assert ape == pytest.approx(0.0, abs=0.03)  # corners: interpolated at shifted stamps
    assert rpe == pytest.approx(0.0, abs=0.03)
    assert full.num_poses == est.num_poses


def test_agreement_reports_offset():
    ref = square_loop()
    est = traj(ref.positions_xyz + np.c_[np.zeros(len(ref.timestamps)), 0.2 * np.sin(np.arange(len(ref.timestamps))
                                                                                         / 10.0),
                                         np.zeros(len(ref.timestamps))],
               np.zeros(len(ref.timestamps)), ref.timestamps)
    ape, _, _ = cse.agreement(ref, est, argparse.Namespace(max_gap=1.0, rpe_delta=2.0))
    assert 0.05 < ape < 0.2


def test_agreement_none_without_overlap():
    ref = square_loop()
    est = traj(ref.positions_xyz, np.zeros(len(ref.timestamps)), ref.timestamps + 1000.0)
    assert cse.agreement(ref, est, argparse.Namespace(max_gap=1.0, rpe_delta=2.0)) == (None, None, None)
