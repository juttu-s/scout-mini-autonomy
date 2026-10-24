#!/usr/bin/env python3
"""Estimate the D455's height, pitch and roll from the floor it sees, for the base_link -> camera_link transform.

    rosrun scout_nav estimate_camera_mount.py [--lidar-height 0.76]

The robot must stand on the floor with open floor in front of the camera (at least ~1 m, nothing on it).
Uses the detector's aligned depth (/camera/camera/aligned_depth_to_color, color optical frame). Fits the floor plane
with RANSAC over 15 frames and prints cam_z / cam_pitch / cam_roll for scout_safety/robot.launch
(base_link = the turning centre, at lidar height, so cam_z = camera height above floor - lidar height). cam_x / cam_y / cam_yaw can't be seen
from the floor: measure x and y with a tape (camera centre relative to the lidar centre, x forward, y left).
"""
import argparse

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


def fit_plane(pts, iters=300, tol=0.02, rng=np.random.default_rng(0)):
    """RANSAC plane: (unit normal n, d) with n.p + d = 0, and the inlier mask."""
    best = None
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs(pts @ n - n @ a) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    q = pts[best]  # refine with least squares on the inliers
    c = q.mean(0)
    n = np.linalg.svd(q - c)[2][-1]
    return n, -n @ c, np.abs(pts @ n - n @ c) < tol


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar-height", type=float, default=0.76)
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--ns", default="/camera/camera/aligned_depth_to_color")
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("estimate_camera_mount", anonymous=True)

    info = rospy.wait_for_message(a.ns + "/camera_info", CameraInfo, 10)
    K = np.array(info.K).reshape(3, 3)
    pts = []
    for _ in range(a.frames):
        m = rospy.wait_for_message(a.ns + "/image_raw", Image, 5)
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
        v, u = np.mgrid[m.height // 2:m.height:4, 0:m.width:4]  # lower half of the image: where the floor is
        z = d[v, u]
        ok = (z > 0.3) & (z < 4.0)
        pts.append(np.stack([(u[ok] - K[0, 2]) * z[ok] / K[0, 0], (v[ok] - K[1, 2]) * z[ok] / K[1, 1], z[ok]], 1))
    p = np.concatenate(pts)
    if len(p) < 2000:
        raise SystemExit("too little depth in the lower half of the image (%d points); is the floor visible?" % len(p))

    n, d, inl = fit_plane(p[np.random.default_rng(1).choice(len(p), min(len(p), 30000), replace=False)])
    if n[1] > 0:  # make the normal point up: in the optical frame (x right, y down, z forward) up is -y
        n, d = -n, -d
    height = d  # distance from the camera origin to the plane (n.0 + d), with n pointing up from the floor
    frac = inl.mean()
    # up vector in camera_link axes (x forward, y left, z up) from the optical frame
    up = np.array([n[2], -n[0], -n[1]])
    # base_link -> camera_link = Rz(yaw) Ry(pitch) Rx(roll); world up seen from the camera is R^T z:
    # pitch only: (-sin p, 0, cos p) (p > 0 = camera tilted down); roll only: (0, sin r, cos r)
    pitch = np.arctan2(-up[0], up[2])
    roll = np.arctan2(up[1], up[2])
    print("floor plane: %.0f %% of %d lower-image points are on it (should be high: open floor in view)" % (100 * frac, len(inl)))
    print("camera height above the floor: %.3f m   pitch %.1f deg (down positive)   roll %.1f deg"
          % (height, np.degrees(pitch), np.degrees(roll)))
    if frac < 0.4:
        print("WARNING: less than 40 %% of points fit the floor; clear the area in front of the camera and rerun")
    print("\nrobot.launch arguments (base_link = MID360 at %.2f m):" % a.lidar_height)
    print("  cam_z:=%.3f cam_pitch:=%.4f cam_roll:=%.4f   (+ measured cam_x:=... cam_y:=...)"
          % (height - a.lidar_height, pitch, roll))


if __name__ == "__main__":
    main()
