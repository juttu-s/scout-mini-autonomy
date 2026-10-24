#!/usr/bin/env python3
"""Find the robot in the map when the start pose is unknown: global search, then verify candidates with NDT.

    rosrun scout_nav relocalize_best.py [--candidates 10] [--settle 3.0]

hdl_localization's /relocalize keeps only the single best global-search result, which in a lab with repeating desk
rows was a different place each call (0.77-0.90 inliers). This asks hdl_global_localization for several candidates,
sets each as the pose estimate (/initialpose), lets hdl_localization's NDT settle, scores it by inlier fraction and
pose steadiness, and finally sets the best one. The robot must stand still. Needs navigation.launch running.
"""
import argparse

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hdl_global_localization.srv import QueryGlobalLocalization
from hdl_localization.msg import ScanMatchingStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import tf2_ros
from sensor_msgs.msg import PointField
from tf.transformations import quaternion_matrix


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def to_frame(cloud, tr):
    """The cloud's x, y, z moved into tr's target frame (plain xyz cloud; the global search uses nothing else)."""
    o = {f.name: f.offset for f in cloud.fields}
    a = np.frombuffer(cloud.data, np.dtype({"names": list("xyz"), "formats": ["<f4"] * 3,
                                            "offsets": [o[k] for k in "xyz"], "itemsize": cloud.point_step}),
                      count=cloud.width * cloud.height)
    p = np.stack([a[k] for k in "xyz"], 1).astype(np.float64)
    q, v = tr.transform.rotation, tr.transform.translation
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    p = (p @ T[:3, :3].T + [v.x, v.y, v.z]).astype(np.float32)
    p = p[np.isfinite(p).all(1)]
    out = PointCloud2(header=cloud.header, height=1, width=len(p), is_bigendian=False, point_step=12,
                      row_step=12 * len(p), is_dense=True,
                      fields=[PointField(n, 4 * i, PointField.FLOAT32, 1) for i, n in enumerate("xyz")])
    out.header.frame_id = tr.header.frame_id
    out.data = p.tobytes()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--candidates", type=int, default=10)
    ap.add_argument("--settle", type=float, default=3.0, help="s to let NDT converge on each candidate")
    ap.add_argument("--cloud", default="/livox/lidar_filtered")
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("relocalize_best")

    scan = rospy.wait_for_message(a.cloud, PointCloud2, 5)
    # the search returns the pose of the cloud's frame: give it the cloud in base_link (the turning centre; the MID360
    # is 0.178 m ahead of it), so the candidates are base_link poses like /initialpose expects
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    tr = buf.lookup_transform("base_link", scan.header.frame_id, rospy.Time(0), rospy.Duration(3))
    scan = to_frame(scan, tr)
    query = rospy.ServiceProxy("/hdl_global_localization/query", QueryGlobalLocalization)
    query.wait_for_service(10)
    res = query(max_num_candidates=a.candidates, cloud=scan)
    cands = list(res.poses)
    print("global search: %d candidates" % len(cands))
    if not cands:
        raise SystemExit("no candidates")

    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    stat, odom = [], []
    rospy.Subscriber("/localization/status", ScanMatchingStatus, lambda m: stat.append(m.inlier_fraction))
    rospy.Subscriber("/localization/odom", Odometry, lambda m: odom.append(
        (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))))
    rospy.sleep(0.5)

    def try_pose(pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = "map", rospy.Time.now()
        msg.pose.pose = pose
        pub.publish(msg)
        rospy.sleep(a.settle)  # converge
        del stat[:], odom[:]
        rospy.sleep(2.0)       # measure
        if not stat or not odom:
            return 0.0, np.inf, None
        o = np.array(odom)
        spread = float(np.hypot(np.ptp(o[:, 0]), np.ptp(o[:, 1])))
        return float(np.mean(stat)), spread, o[-1]

    results = []
    for i, p in enumerate(cands):
        inl, spread, final = try_pose(p)
        results.append((inl, -spread, i, final))
        print("  #%d search (%.2f, %.2f, %4.0f deg) inlier %.2f -> NDT inliers %.3f, spread %.2f m%s" % (
            i, p.position.x, p.position.y, np.degrees(yaw_of(p.orientation)), res.inlier_fractions[i], inl, spread,
            "" if final is None else ", at (%.2f, %.2f, %.0f deg)" % (final[0], final[1], np.degrees(final[2]))))
    best = max(results, key=lambda r: (r[0] - 0.5 * max(-r[1] - 0.05, 0.0)))  # inliers, penalize unsteady poses
    i = best[2]
    inl, spread, final = try_pose(cands[i])
    print("best: candidate #%d -> inliers %.3f, spread %.2f m, pose (%.2f, %.2f, %.0f deg)%s" % (
        i, inl, spread, final[0], final[1], np.degrees(final[2]),
        "" if inl > 0.95 else "   (below 0.95: check the robot's position on the map)"))


if __name__ == "__main__":
    main()
