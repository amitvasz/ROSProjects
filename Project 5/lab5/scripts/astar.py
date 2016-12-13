#!/usr/bin/env python
import roslib
import rospy
import rosbag
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys
from astar_util import *

roslib.load_manifest('lab5')

rospy.set_param('goalX',4)
rospy.set_param('goalY',9)

class Path(object):
    def __init__(self):
        self.i = 0.0
        self.j = 0.0

z = 0
counter = 0
old_covered_angle = 0.0
covered_angle = 0.0

def getCoveredAngle(odom):
    global covered_angle
    global old_covered_angle
    global z

    z = 2 * math.asin(odom.pose.pose.orientation.z)
    covered_angle = z - old_covered_angle

def GetDataFromFile(mapFile):

    rospy.Subscriber("/odom", Odometry, getCoveredAngle)

    map_file_handler = open(mapFile).read().split("\n")
    data = []
    for line in map_file_handler:
        points = [int(i) for i in line.split()]
        data.append(points)

    GetPath(data)

def GetPath(data):

    si = 11
    sj = 1
    di = 10 - int(rospy.get_param('goalY'))
    dj = 9 + int(rospy.get_param('goalX'))
    if data[dj][di] == 1:
        rospy.logerr("Given coordinate contains an obstacle, change coordinate and try again")
        exit(0)

    astar = AStar(data,(sj,si),(dj,di))
    path = astar.solve()

    stri = "\n\n"
    for i in xrange(len(data)):
        for j in xrange(len(data[0])):
            if (j,i) in path:
                stri += "*"
            else:
                stri += str(data[i][j])
        stri += "\n"
    rospy.logerr(stri)

    if path is None:
        rospy.logerr("PATH NOT FOUND OR NOT POSSIBLE")
        exit(0)

    robotPath = []
    for coords in path:
        temp = Path()
        temp.j = coords[0]
        temp.i = coords[1]
        rospy.logerr("%d %d",temp.i,temp.j)
        robotPath.append(temp)
        del temp

    Motion(robotPath,len(path))

def Move(distance):
    rate = rospy.Rate(10)   
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10000)

    msg = Twist()
    count=12
    while count > 0:
        msg.linear.x = (distance*2)
        pub.publish(msg)
        rate.sleep()
        count -= 1

def Rotate(angle, dir):
    global covered_angle
    global old_covered_angle

    if angle <= 0.004:
        return

    rate = rospy.Rate(10)   
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10000)

    covered_angle = 0.0
    
    while angle - math.fabs(covered_angle) > 0.004:
        msg = Twist()
        if angle - math.fabs(covered_angle) > 0.08:        
            velocity = 0.2
        else:
            velocity = 0.01
        if dir == 0:
            msg.angular.z = velocity        
        elif dir == 1:
            msg.angular.z = (-1)*velocity
        
        pub.publish(msg)
        rate.sleep()
    
    old_covered_angle += covered_angle


def Motion(path, size):
    global z

    i=0
    current_i = path[0].i
    current_j = path[0].j
    
    for i in range(1,size+1):

        if path[i].j-current_j == -1:
            angle = math.pi
            distance = 1
            angle = -(z-angle)
        elif path[i].j-current_j == 1:
            angle = 0
            distance = 1
            angle = -(angle+z)
        elif path[i].j-current_j == 0:
            if path[i].i-current_i == -1:
                angle = ((math.pi/2)-z) 
                distance = 1
            else:
                if path[i].i-current_i == 1:
                    angle = -((math.pi/2)+z)    
                    distance = 1

        if angle<0:
            Rotate(-angle,1)
        else:
            Rotate(angle,0)
        Move(distance)

        current_i = path[i].i
        current_j = path[i].j        

if __name__ == "__main__":
    try:
        rospy.init_node('astar')
        GetDataFromFile(sys.argv[1])
    except rospy.ROSInterruptException:
        sys.exit(0)
    rospy.spin()