#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from math import *
import pyproj
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Node():
    def __init__(self):
        self.lat=0
        self.long = 0
        self.datum=[-33.839404,151.254627]
        self.last_x = 0
        self.last_y = 0
        self.last_time = 0
        self.orientation = None
        self.dt = 0.05
        self.angular_vel = 0

        self.odom_pub = rospy.Publisher('odom',Odometry,queue_size = 10)
        self.state = np.matrix('0.0;0;0;0;0;0') #[x,y,yaw,vx,vy,yaw_rate]
        self.state_covariance = np.matrix('1.0,0.0,0.0,0.0,0.0,0.0;\
                                           0.0,1.0,0.0,0.0,0.0,0.0;\
                                           0.0,0.0,1.0,0.0,0.0,0.0;\
                                           0.0,0.0,0.0,1.0,0.0,0.0;\
                                           0.0,0.0,0.0,0.0,1.0,0.0;\
                                           0.0,0.0,0.0,0.0,0.0,1.0')

    def predict(self):

        old = self.state
        self.state = self.state
        #self.state[0] = self.state[0] + self.state[3]*self.dt
        #self.state[1] = self.state[1] + self.state[4]*self.dt
        #self.state[2] = self.state[2] + self.state[5]*self.dt
        print("A",self.state[2])
        F = np.matrix([[1.0,0,0,self.dt,0,0],
                       [0,1,0,0,self.dt,0],
                       [0,0,1.0,0,0,self.dt],
                       [0,0,0,1,0,0],
                       [0,0,0,0,1,0],
                       [0,0,0,0,0,1]])
        self.state = np.matmul(F,self.state)

        print("B",self.state[2])
        Q = np.matrix('.01,0.0,0.0,0.0,0.0,0.0;\
                       0.0,0.1,0.0,0.0,0.0,0.0;\
                       0.0,0.0,0.01,0.0,0.0,0.0;\
                       0.0,0.0,0.0,.01,0.0,0.0;\
                       0.0,0.0,0.0,0.0,.01,0.0;\
                       0.0,0.0,0.0,0.0,0.0,.01')
        self.state_covariance = np.matmul(np.matmul(F,self.state_covariance),np.transpose(F)) + Q
        #print(self.state)
        #print(self.state_covariance)

    def gps_update(self,x,y):

        gps_covariance = np.matrix('1,0;\
                                    0,1')
        K = np.matmul(self.state_covariance[:2,:2],np.linalg.inv(self.state_covariance[:2,:2]+gps_covariance))
        self.state[:2] = self.state[:2] + np.matmul(K,np.matrix([[x],[y]])-self.state[:2])
        self.state_covariance[:2,:2] = self.state_covariance[:2,:2] - np.matmul(K,self.state_covariance[:2,:2])

        now_time = rospy.get_time()
        if (self.last_time!=0):
            dtime = now_time-self.last_time
            self.state[3] = (x - self.last_x)/dtime
            self.state[4] = (y - self.last_y)/dtime

        self.last_time = now_time
        self.last_x = x
        self.last_y = y

        #print(self.state)

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
        # odom_msg=Odometry()
        # odom_msg.pose.pose.position.x = x
        # odom_msg.pose.pose.position.y = y
        # odom_msg.pose.pose.position.z = 0.0
        # odom_msg.pose.pose.orientation = self.orientation
        # odom_msg.header.frame_id = "odom"
        # self.odom_pub.publish(odom_msg)
        #print(x,y)
        self.gps_update(x,y)

    def imu_update(self,yaw,yaw_rate):
        imu_covariance = np.matrix('0.01,0;\
                                    0,0.01')

        #K = self.state_covariance[2,2] / (self.state_covariance[2,2]+0)
        #self.state[2] = self.state[2] + K*(yaw-self.state[2])
        #self.state_covariance[2,2] = self.state_covariance[2,2] - K*self.state_covariance[2,2]
        #print(self.state[2],yaw)
        K = np.matmul(self.state_covariance[(2,5),:][:,(2,5)],np.linalg.inv(self.state_covariance[(2,5),:][:,(2,5)]+imu_covariance))
        self.state[2:6:3] = self.state[2:6:3] + np.matmul(K,np.matrix([[yaw],[yaw_rate]])-self.state[2:6:3])
        self.state_covariance[(2,5),:][:,(2,5)] = self.state_covariance[(2,5),:][:,(2,5)] - np.matmul(K,self.state_covariance[(2,5),:][:,(2,5)])

    def imu_callback(self,data):
        self.orientation = data.orientation
        orientation_q = data.orientation
        orientation_list =[orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        yaw_rate = data.angular_velocity.z
        self.imu_update(yaw+0.218,yaw_rate)
        #print(orientation)

    def publish(self):
        print("C",self.state[2])
        odom_msg=Odometry()
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0
        yaw = self.state[2].copy()
        q = quaternion_from_euler(0,0,yaw)


        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.header.frame_id = "odom"
        self.odom_pub.publish(odom_msg)
        print("D",self.state[2])
        #print(self.state[2])



if __name__ == "__main__":
    rospy.init_node("simple_gps_to_odom")
    rate = rospy.Rate(20)
    node = Node()
    rospy.Subscriber("gps",NavSatFix,node.gps_callback)
    rospy.Subscriber("imu",Imu,node.imu_callback)
    while not rospy.is_shutdown():
        node.predict()
        node.publish()
        rate.sleep()
