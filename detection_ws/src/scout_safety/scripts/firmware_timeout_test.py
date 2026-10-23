#!/usr/bin/env python3
"""Does the Scout stop by itself when /cmd_vel commands stop arriving?

Run with ONLY the Scout base running (no safety node, no teleop) and the WHEELS OFF THE GROUND:
    rosrun scout_safety firmware_timeout_test.py
It drives at 0.1 m/s for 3 s, then stops publishing without sending a stop command, and watches the
wheel speed in /scout_status. If the wheels are still turning after 3 s it sends zero commands itself.
"""
import sys
import time

import rosgraph
import rospy
from geometry_msgs.msg import Twist
from scout_msgs.msg import ScoutStatus

MODES = {0: "STANDBY", 1: "CAN (computer control)", 2: "UART", 3: "RC (handheld remote)"}
STATES = {0: "NORMAL", 1: "E-STOP", 2: "EXCEPTION"}
ERRORS = {0x0001: "battery fault", 0x0002: "battery warning", 0x0004: "RC signal lost", 0x0008: "motor 1 comm",
          0x0010: "motor 2 comm", 0x0020: "motor 3 comm", 0x0040: "motor 4 comm", 0x0100: "motor driver",
          0x0200: "high-level comm"}


def main():
    rospy.init_node("firmware_timeout_test")
    status = rospy.wait_for_message("/scout_status", ScoutStatus, timeout=5.0)
    faults = [n for bit, n in ERRORS.items() if status.fault_code & bit]
    print("Scout: state %s, control mode %s, faults: %s" % (STATES.get(status.base_state, status.base_state),
          MODES.get(status.control_mode, status.control_mode), ", ".join(faults) or "none"))
    if status.control_mode != 1 or status.base_state != 0:
        print("The base will not accept /cmd_vel like this. Switch the remote to command/CAN mode "
              "(control mode must be CAN, state NORMAL) and run again.")
        return 1

    pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    others = [n for t, nodes in pubs if t == "/cmd_vel" for n in nodes]
    if others:
        print("Other nodes publish /cmd_vel (%s); stop them first." % ", ".join(others))
        return 1

    if input("Wheels OFF the ground and e-stop within reach? Type yes: ").strip().lower() != "yes":
        return 1

    speed = []
    rospy.Subscriber("/scout_status", ScoutStatus, lambda m: speed.append((time.time(), m.linear_velocity)))
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.5)
    cmd = Twist()
    cmd.linear.x = 0.1
    print("Driving at 0.1 m/s for 3 s ...")
    t_end = time.time() + 3.0
    while time.time() < t_end and not rospy.is_shutdown():
        pub.publish(cmd)
        rospy.sleep(0.1)
    moving = [v for t, v in speed if t > t_end - 1.0]
    print("Wheel speed while commanded: %.3f m/s" % (sum(moving) / max(len(moving), 1)))

    print("Stopped publishing (no stop command sent). Watching for 3 s ...")
    t_stop = time.time()
    rospy.sleep(3.0)
    after = [(t - t_stop, v) for t, v in speed if t > t_stop]
    stopped_at = next((t for t, v in after if abs(v) < 0.01), None)
    for t, v in after[::max(1, len(after) // 15)]:
        print("  +%.2f s  %.3f m/s" % (t, v))
    if stopped_at is not None and all(abs(v) < 0.01 for t, v in after if t >= stopped_at):
        print("RESULT: the base stops by itself %.2f s after commands stop -> a crashed safety node is fail-safe."
              % stopped_at)
    else:
        print("RESULT: the base kept moving without commands -> NOT fail-safe; sending stop now.")
        t_end = time.time() + 1.0
        while time.time() < t_end:
            pub.publish(Twist())
            rospy.sleep(0.05)
    return 0


if __name__ == "__main__":
    sys.exit(main())
