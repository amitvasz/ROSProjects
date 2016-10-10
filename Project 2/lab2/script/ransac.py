#!/usr/bin/env python
__author__ = "Moonis Javed"

import roslib
import rospy
import random
import math
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

roslib.load_manifest('lab2')

ranges_len = 361

def transformRanges(range_array):
    global sinx
    global cosx
    range_array = np.array(range_array)
    range_array[range_array == 3.0] = 0
    y = range_array * sinx
    x = range_array * cosx
    return x,y

def getCoordinates(data):
    global ranges
    x,y = transformRanges(data.ranges)
    ranges[:,0:1] = x.reshape((x.shape[0], 1))
    ranges[:,1:] = y.reshape((y.shape[0], 1))

def getDistance(x0,x1,x2,y0,y1,y2):
    line_distance = abs((y2 - y1) * x0 - (x2 - x1)*y0 + x2 * y1 - y2 * x1) 
    length = math.sqrt((y2 - y1) **2 + (x2 - x1) **2)
    return line_distance/length

def ransac(x, y, num_iterations, threshold, point_threshold, num_cycles):
    global X_Array, Y_Array
    x = ranges[:,0]
    y = ranges[:,1]
    index_array = [i for i in xrange(ranges_len)]
    X_Array = [-1]
    Y_Array = [-1]


    for _ in xrange(num_cycles):
        inliers = []
        outliers = []

        for i in xrange(num_iterations):
            temp_inl = []
            temp_outl = []
            
            index1 = random.randint(0,len(index_array)-1)
            index2 = random.randint(0,len(index_array)-1)
            if index1 == index2:
                continue
            x1 = x[index_array[index1]]
            y1 = y[index_array[index1]]
            
            x2 = x[index_array[index2]]
            y2 = y[index_array[index2]]
            
            if x1 != 0 and x2 != 0:
                for j in index_array:
                    x0 = x[j]
                    y0 = y[j]
                    if x0 != 0 or y0 != 0:
                        dist = getDistance(x0,x1,x2,y0,y1,y2)
                        if dist < threshold:
                            temp_inl.append(j)
                        else:
                            temp_outl.append(j)

            if len(inliers) < len(temp_inl):
                inliers = temp_inl
                outliers = temp_outl
                max_x = np.max(ranges[inliers,0]) 
                end_x = np.min(ranges[inliers,0])
                end_point = ranges[inliers,:]
                max_y = end_point[np.argmax(end_point[:,0]), 1]
                end_y = end_point[np.argmin(end_point[:,0]), 1]
                in1 = index1
                in2 = index2
            
        if len(inliers) > point_threshold:
            X_Array.append(max_x)
            Y_Array.append(max_y)
            X_Array.append(end_x)
            Y_Array.append(end_y)
            index_array = outliers

def visualize():
    global ranges
    rate = rospy.Rate(10)
    pub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
    pub1 = rospy.Publisher('actual_vis', Marker, queue_size=10)
    num_iterations = 50
    threshold = 0.1
    point_threshold = 3
    x = 1
    y = 1
    num_cycles = 10

    while not rospy.is_shutdown():
        try:
            ransac(x, y, num_iterations, threshold, point_threshold, num_cycles)
            
            marker = Marker()
            marker1 = Marker()
            marker.header.frame_id = "/odom"
            marker.header.stamp = rospy.Time.now()
            marker1.header.frame_id = "/odom"
            marker1.header.stamp = rospy.Time.now()
            marker.ns = "ransac"
            marker1.ns = "object"
            marker.id = 0
            marker1.id = 1
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker1.type = Marker.LINE_STRIP
            marker1.action = Marker.ADD
            marker.scale.x = 0.1
            marker.color.r = 1.0
            marker.color.a = 1.0           
            marker1.scale.x = 0.1
            marker1.color.b = 1.0
            marker1.color.a = 1.0
            marker.lifetime = rospy.Duration()
            marker1.lifetime = rospy.Duration()

            for i in range(1,len(X_Array)):
                p = Point()
                p.x = X_Array[i]
                p.y = Y_Array[i]
                marker.points.append(p)

            for i in range(ranges_len):
                if ranges[i,0] == 0 and ranges[i,1] == 0:
                    continue
                p = Point()
                p.x = ranges[i,0]
                p.y = ranges[i,1]
                marker1.points.append(p)

            pub.publish(marker)
            pub1.publish(marker1)
       
        except:
            continue		
            
if __name__ == '__main__':
    try:
        rospy.init_node('ransac', anonymous=True)
        ranges = np.zeros((ranges_len,2))
        rospy.Subscriber("/robot_0/base_scan", LaserScan, getCoordinates)
        degrees = np.linspace(np.pi/2,-1 * np.pi / 2 , ranges_len)
        sinx = np.sin(degrees)
        cosx = np.cos(degrees)
        visualize()
    except rospy.ROSInterruptException:
        pass
