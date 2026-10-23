#!/usr/bin/env python3
"""Clear move_base's costmaps after every relocalization (/initialpose).

While localization is wrong (before a pose estimate, or during a global search), the costmaps' obstacle layers
stamp live scans at wrong places. Nothing ray-traces through those places later, so they stay as ghost walls that
box the robot in ("NO PATH", lab2 2026-10-18 and again 2026-10-19). After a new pose estimate, wait for NDT to
settle, then clear.
"""
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty


def main():
    rospy.init_node("clear_on_relocalize")
    delay = rospy.get_param("~delay", 3.0)
    clear = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    pending = []
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, lambda m: pending.append(rospy.get_time()))
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if pending and rospy.get_time() - pending[-1] >= delay:
            del pending[:]
            try:
                clear()
                rospy.loginfo("New pose estimate: cleared move_base costmaps (ghost obstacles from the old pose)")
            except rospy.ServiceException as e:
                rospy.logwarn("clear_costmaps failed: %s", e)
        rate.sleep()


if __name__ == "__main__":
    main()
