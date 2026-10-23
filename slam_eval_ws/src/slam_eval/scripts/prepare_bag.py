#!/usr/bin/env python3
"""Write the derived topics the SLAM methods need, once per recorded session (offline, before any timing run).

    rosrun slam_eval prepare_bag.py ~/maps/<name> [--out eval/derived.bag]

The recording has the MID360 as livox_ros_driver2 PointCloud2 (x y z intensity tag line timestamp[f64 ns]) and its
6-axis IMU in g. DLIO and RTAB-Map take those as they are; the others need, written here so that no conversion runs
(and costs CPU) while a method is being timed:
  /livox/custom       livox_ros_driver2/CustomMsg (FAST-LIO's Livox mode), same points and order
  /livox/points_ring  x y z intensity ring(u16, = line) time(f32 s since scan start), zero points removed,
                      sorted by time (LIO-SAM: dense cloud, last point = scan end)
  /livox/imu_ms2      IMU in m/s^2 with a gravity-aligned orientation from a complementary filter (LIO-SAM wants a
                      9-axis IMU; yaw is integrated gyro, so it only drifts slowly and LIO-SAM's imuRPYWeight is low)
run_slam_eval.py plays this bag together with the original ones (rosbag play merges them by time).
"""
import argparse
import glob
import os
import struct

import numpy as np
import rosbag
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu, PointCloud2, PointField

LIDAR, IMU = "/livox/lidar", "/livox/imu"
G = 9.80665
RING_FIELDS = [PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1),
               PointField("z", 8, PointField.FLOAT32, 1), PointField("intensity", 12, PointField.FLOAT32, 1),
               PointField("ring", 16, PointField.UINT16, 1), PointField("time", 18, PointField.FLOAT32, 1)]
RING_DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("ring", "<u2"), ("time", "<f4")])
LIVOX_PT = np.dtype([("offset_time", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                     ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1")])  # packed, 19 bytes as serialized


def session_bags(d):
    return sorted(glob.glob(os.path.join(d, "recording", "*.bag")), key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]))


def livox_points(msg):
    offs = {f.name: f.offset for f in msg.fields}
    names = ["x", "y", "z", "intensity", "tag", "line", "timestamp"]
    dt = np.dtype({"names": names, "formats": ["<f4"] * 4 + ["u1", "u1", "<f8"],
                   "offsets": [offs[n] for n in names], "itemsize": msg.point_step})
    return np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)


def header_bytes(h):
    f = h.frame_id.encode()
    return struct.pack("<III", h.seq, h.stamp.secs, h.stamp.nsecs) + struct.pack("<I", len(f)) + f


def custom_raw(msg, a):
    """CustomMsg serialized directly with numpy (building ~20k Python point objects per scan is far too slow)."""
    base = int(a["timestamp"][0]) if len(a) else msg.header.stamp.to_nsec()
    p = np.empty(len(a), LIVOX_PT)
    p["offset_time"] = np.clip(a["timestamp"] - base, 0, None).astype(np.uint32)
    for k in "xyz":
        p[k] = a[k]
    p["reflectivity"] = np.clip(a["intensity"], 0, 255).astype(np.uint8)
    p["tag"], p["line"] = a["tag"], a["line"]
    data = header_bytes(msg.header) + struct.pack("<QIB3B", base, len(a), 0, 0, 0, 0) + struct.pack("<I", len(a)) + p.tobytes()
    return (CustomMsg._type, data, CustomMsg._md5sum, None, CustomMsg)


def ring_cloud(msg, a):
    a = a[(a["x"] != 0) | (a["y"] != 0) | (a["z"] != 0)]
    a = a[np.argsort(a["timestamp"], kind="stable")]
    p = np.empty(len(a), RING_DT)
    for k in "xyz":
        p[k] = a[k]
    p["intensity"], p["ring"] = a["intensity"], a["line"]
    p["time"] = ((a["timestamp"] - msg.header.stamp.to_nsec()) * 1e-9).astype(np.float32)
    out = PointCloud2(header=msg.header, height=1, width=len(p), fields=RING_FIELDS, is_bigendian=False,
                      point_step=RING_DT.itemsize, row_step=RING_DT.itemsize * len(p), is_dense=True)
    out.data = p.tobytes()
    return out


class Attitude:
    """Complementary filter: gyro integration, roll/pitch pulled towards gravity when |acc| is close to 1 g."""

    def __init__(self, bias, k=0.005):
        self.q, self.t, self.k, self.bias = None, None, k, bias

    def update(self, t, gyro, acc_ms2):
        if self.q is None:
            ax, ay, az = acc_ms2 / np.linalg.norm(acc_ms2)
            r, p = np.arctan2(ay, az), np.arctan2(-ax, np.hypot(ay, az))
            cr, sr, cp, sp = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2)
            self.q, self.t = np.array([cr * cp, sr * cp, cr * sp, -sr * sp]), t  # w x y z, yaw 0
            return self.q
        dt, self.t = t - self.t, t
        w = np.asarray(gyro, float) - self.bias
        n = np.linalg.norm(acc_ms2)
        if abs(n - G) < 0.1 * G:  # correction: rotate the measured gravity direction onto world +z
            qw, qx, qy, qz = self.q
            Rq = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                           [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                           [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
            up_body = Rq.T @ np.array([0.0, 0.0, 1.0])
            w = w + (self.k / max(dt, 1e-3)) * np.cross(acc_ms2 / n, up_body)
        qw, qx, qy, qz = self.q
        dq = 0.5 * np.array([-qx * w[0] - qy * w[1] - qz * w[2], qw * w[0] + qy * w[2] - qz * w[1],
                             qw * w[1] - qx * w[2] + qz * w[0], qw * w[2] + qx * w[1] - qy * w[0]])
        self.q = self.q + dq * dt
        self.q /= np.linalg.norm(self.q)
        return self.q


def gyro_bias(bag, window):
    """Mean gyro over the first `window` s; the robot stands still at the start of a session (lab2: 0.015 rad/s on
    z, i.e. 0.9 deg/s of yaw drift without it)."""
    g, t0 = [], None
    with rosbag.Bag(bag) as ib:
        for _, m, _ in ib.read_messages(topics=[IMU]):
            t0 = t0 or m.header.stamp.to_sec()
            if m.header.stamp.to_sec() - t0 > window:
                break
            g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    g = np.array(g)
    if g.std(0).max() > 0.02:
        raise SystemExit("gyro not still in the first %.0f s (std %s): pass a shorter --bias-window" % (window, g.std(0)))
    return g.mean(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir")
    ap.add_argument("--out", default="eval/derived.bag")
    ap.add_argument("--bias-window", type=float, default=3.0, help="s of standing still at the start")
    a = ap.parse_args()
    d = os.path.expanduser(a.session_dir)
    out = os.path.join(d, a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    bags = session_bags(d)
    bias = gyro_bias(bags[0], a.bias_window)
    print("gyro bias %s rad/s" % np.round(bias, 4))
    att = Attitude(bias)
    n_scan = n_imu = 0
    with rosbag.Bag(out + ".tmp", "w") as ob:
        for b in bags:
            with rosbag.Bag(b) as ib:
                for topic, m, t in ib.read_messages(topics=[LIDAR, IMU]):
                    if topic == LIDAR:
                        pts = livox_points(m)
                        ob.write("/livox/custom", custom_raw(m, pts), t, raw=True)
                        ob.write("/livox/points_ring", ring_cloud(m, pts), t)
                        n_scan += 1
                        if n_scan % 500 == 0:
                            print("\r  %d scans" % n_scan, end="", flush=True)
                    else:
                        acc = np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]) * G
                        gyr = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
                        q = att.update(m.header.stamp.to_sec(), gyr, acc)
                        o = Imu(header=m.header, angular_velocity=m.angular_velocity,
                                angular_velocity_covariance=m.angular_velocity_covariance,
                                linear_acceleration_covariance=m.linear_acceleration_covariance)
                        o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z = acc
                        o.orientation.w, o.orientation.x, o.orientation.y, o.orientation.z = q
                        o.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.1]
                        ob.write("/livox/imu_ms2", o, t)
                        n_imu += 1
    os.replace(out + ".tmp", out)
    print("\r%s: %d scans, %d IMU messages" % (out, n_scan, n_imu))


if __name__ == "__main__":
    main()
