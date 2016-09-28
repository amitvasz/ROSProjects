#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import math
import message_filters

roslib.load_manifest('lab1')

def pursuer_broadcast(odom1,odom2):

    rate = rospy.Rate(10.0)
    now = rospy.Time.now()

    br = tf.TransformBroadcaster()
    br.sendTransform((odom2.pose.pose.position.x, odom2.pose.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, odom2.pose.pose.orientation.z),
                     rospy.Time.now(),
                     "/robot_1/odom",
                     "/world")


if __name__ == "__main__":
    try:
        rospy.init_node("pursuer", anonymous=False)
        sub = message_filters.Subscriber('/robot_0/odom', Odometry)
        sub2 = message_filters.Subscriber('/robot_1/odom', Odometry)
        ts = message_filters.TimeSynchronizer([sub, sub2], 10)
        ts.registerCallback(pursuer_broadcast)

        pub = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=1)

        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():
            # rospy.logerr("tested")
            listener.waitForTransform('/robot_1/odom','/robot_0/odom',rospy.Time.now(),rospy.Duration(5.0))
            try:
                (trans, rot) = listener.lookupTransform('/robot_1/odom', '/robot_0/odom', rospy.Time())
            except tf.LookupException:
                continue
            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            
            pub.publish(msg)
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass