#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from math import *
import pyproj

class Node():
    def __init__(self):
        self.lat=0
        self.long = 0
        self.datum=[-33.839404,151.254627]
        self.x = 0
        self.y = 0
        self.orientation = None

        self.odom_pub = rospy.Publisher('odom',Odometry,queue_size = 10)




    def gps_callback(self,data):
        self.lat = data.latitude
        self.long = data.longitude
        R = 6378100 #Radius of Earth Metres
        pi=3.14159265359
        #delta_lat = self.lat-self.datum[0]
        #delta_long = self.long-self.datum[1]
        #y = (delta_lat/360 )* 2*pi*R
        #x = (delta_long/360 )* 2*pi*
        #print(self.lat,self.long)

        outProj = pyproj.Proj("+init=EPSG:4326")
        crs="+proj=tmerc +lon_0={} +lat_0={} +units=m".format(self.datum[1],self.datum[0])
        inp = pyproj.Proj(crs)
        y,x = pyproj.transform(outProj,inp,self.long,self.lat)
        #print(outProj)
        odom_msg=Odometry()
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.orientation
        odom_msg.header.frame_id = "odom"
        self.odom_pub.publish(odom_msg)
        print(x,y)

    def imu_callback(self,data):
        self.orientation = data.orientation
        #print(orientation)




if __name__ == "__main__":
    rospy.init_node("simple_gps_to_odom")
    rate = rospy.Rate(20)
    node = Node()
    rospy.Subscriber("gps",NavSatFix,node.gps_callback)
    rospy.Subscriber("imu",Imu,node.imu_callback)
    while not rospy.is_shutdown():
            rate.sleep()
