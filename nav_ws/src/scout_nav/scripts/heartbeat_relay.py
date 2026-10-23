#!/usr/bin/env python3
"""Health signal for the scout-base container: touches a file once a second while the wheel odometry (/odom, i.e.
the Scout talks over CAN) and the MID360 cloud (/livox/lidar) are both fresh. One persistent node, so the Docker
health check is a file-age test and never starts a ROS client of its own.

    heartbeat_relay.py [--file /run/scout/base.alive] [--max-age 1.0] [topic ...]
"""
import argparse
import os
import time

import rospy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="/run/scout/base.alive")
    ap.add_argument("--max-age", type=float, default=1.0)
    ap.add_argument("topics", nargs="*", default=["/odom", "/livox/lidar"])
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("base_heartbeat")
    last = {t: 0.0 for t in a.topics}
    for t in a.topics:
        rospy.Subscriber(t, rospy.AnyMsg, lambda _m, t=t: last.__setitem__(t, time.monotonic()), queue_size=1)
    os.makedirs(os.path.dirname(a.file), exist_ok=True)
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        now = time.monotonic()
        if all(now - v < a.max_age for v in last.values()):
            with open(a.file, "a"):
                os.utime(a.file, None)
        r.sleep()


if __name__ == "__main__":
    main()
