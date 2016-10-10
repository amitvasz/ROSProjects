#!/usr/bin/env python
__author__ = "Moonis Javed"

import roslib
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

roslib.load_manifest('lab2')


def checkFrontAndLeftWall(data):
    global front_wall, left_wall
    range_array = data.ranges
    counter_front = 0
    counter_left = 0

    for i in xrange(120):
        if range_array[120+i] < 1:
            counter_front += 1

    for i in xrange(70):
        if range_array[i] < 1:
            counter_left += 1

    front_wall = counter_front > 1
    left_wall = counter_left > 10
            
def setOrienAndPos(data):
    global orientation
    global position
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

def getArea(A,B,C):
    return abs(0.5*((A[0]-C[0])*(B[1]-A[1]) - (A[0]-B[0])*(C[1]-A[1])))

def pointOnLine(points,threshold):
    global position
    A = points[0,:]
    B = points[1,:]
    C = np.array([position.x, position.y])
    area = getArea(A,B,C)
    return area < threshold

def getAngleAndDistance(points,x,y,robot_angle):
    dist_temp = math.sqrt((points[1, 0] - x) ** 2 + (points[1, 1] - y) ** 2)
    goal_angle = math.atan((points[1, 1] - y) / (points[1,0]- x)) - 2*robot_angle
    return dist_temp,goal_angle

def bug2():
    global front_wall, orientation, pos, robot_on_line
    pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size = 10)
    sub_truth = rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, setOrienAndPos)
    rate = rospy.Rate(10)
    distance = math.sqrt(9.0 ** 2 + 4.5 ** 2)
    goalReached = False
    dist_threshold = 0.8
    follow_flag = "GOAL_SEEK"
    while not goalReached:

        if orientation != 0:
            
            robot_angle = np.arcsin(orientation.z)
            dist_temp,goal_angle = getAngleAndDistance(points,position.x,position.y,robot_angle)
            
            robot_on_line = pointOnLine(points,dist_threshold)

            speed = Twist()
            if dist_temp < dist_threshold:
                speed.linear.x = 0 
                speed.angular.z = 0
                goalReached = True
                break
            
            else:
                speed.linear.x = 0.0 if front_wall else 4.0
                
                angular_speed = 0
                
                if follow_flag == "GOAL_SEEK":
                    if robot_on_line:
                        angular_speed = min(goal_angle, 1)
                    elif front_wall:
                        angular_speed = 1
                    else:
                        angular_speed = min(goal_angle, 1)
                    speed.angular.z =  angular_speed
                    if front_wall or left_wall:
                        follow_flag = "WALL_FOLLOW"
                else:
                    if front_wall:
                        angular_speed = 0.5
                    else:
                        if left_wall:
                            angular_speed = 0
                        else:
                            angular_speed = -1 * 0.8
                    speed.angular.z = -1 * angular_speed
                    if robot_on_line and not front_wall:
                        follow_flag = "GOAL_SEEK"

            pub.publish(speed)
            rate.sleep()    
          
if __name__ == '__main__':
    try:
        rospy.init_node('bug2', anonymous=True)
        orientation = 0
        points = np.array([[-8, -2],[4.5, 9.0]])
        front_wall = False
        left_wall = False
        robot_on_line = False
        rospy.init_node('bug2', anonymous=True)
        rospy.Subscriber("/robot_0/base_scan", LaserScan, checkFrontAndLeftWall)
        bug2()
    except rospy.ROSInterruptException:
        pass
