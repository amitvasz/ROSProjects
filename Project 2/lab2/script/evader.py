#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

roslib.load_manifest("lab2")

def evader(scan):
    pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=10)
    ranges = scan.ranges
    # print min(ranges[120:240])
    distance = scan.range_max
    obstacle_present = 0.8 > min(ranges)
    # print obstacle_present
    msg = Twist()

    if not obstacle_present:
        msg.linear.x = 2

    else:
        msg.angular.z = 1
    pub.publish(msg)


if __name__ == "__main__":
    try:
        rospy.init_node("evader", anonymous=False)
        rospy.Subscriber("robot_0/base_scan", LaserScan, evader)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass