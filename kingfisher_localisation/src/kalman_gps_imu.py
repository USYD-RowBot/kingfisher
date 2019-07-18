#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from math import *

class Node():
    def __init__(self):
        self.lat=0
        self.long = 0
        self.datum=[-33.886731,151.1992591]
        self.x = 0
        self.y = 0
        self.orientation = None
        self.odom_pub = None



    def gps_callback(self,data):
        self.lat = data.latitude
        self.long = data.longitude
        R = 63781000 #Radius of Earth Metres
        pi=3.14159265359
        # delta_lat = self.lat-self.datum[0]
        # delta_long = self.lat-self.datum[1]
        # x = (delta_lat/360 )* 2*pi*R
        # y = (delta_lat/360 )* 2*pi*R
        # print(x,y)

    def imu_callback(self,data):
        # self.orientation = data.orientation
        # print(orientation)




if __name__ == "__main__":
    rospy.init_node("kalman_gps_to_odom")
    rate = rospy.Rate(20)
    node = Node()
    node.publius
    rospy.Subscriber("gps",NavSatFix,node.gps_callback)
    while not rospy.is_shutdown():
            rate.sleep()
