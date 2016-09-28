#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
import math
import message_filters

roslib.load_manifest('lab1')

def evader(scan,odom):
    pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=10)
    ranges = scan.ranges
    distance = scan.range_max
    obstacle_present = 0.8 > min(ranges)
    msg = Twist()

    if not obstacle_present:
        msg.linear.x = 2

    else:
        msg.angular.z = 1

    br = tf.TransformBroadcaster()
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, odom.pose.pose.orientation.z),
                     rospy.Time.now(),
                     "/robot_0/odom",
                     "/world")
    pub.publish(msg)


if __name__ == "__main__":
    try:
        rospy.init_node('pursuer-evader', anonymous=False)  
        sub = message_filters.Subscriber('/robot_0/base_scan', LaserScan)
        sub2 = message_filters.Subscriber('/robot_0/odom', Odometry)
        ts = message_filters.TimeSynchronizer([sub, sub2], 10)
        ts.registerCallback(evader)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass