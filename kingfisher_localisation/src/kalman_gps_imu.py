#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from math import *
import numpy as np

class Node():
    def __init__(self):
        self.lat=0
        self.long = 0
        self.datum=[-33.886731,151.1992591]
        self.x = 0
        self.y = 0
        self.orientation = None
        self.odom_pub = None

    def kalman_predict(self, dt, x, y, yaw, lin_vel, ang_vel,state_noise, vel_noise, mod_noise):
        # transition matrix
        fx = np.array([1, 0, -dt*lin_vel*np.sin(yaw)],
                      [0, 1,  dt*lin_vel*np.cos(yaw)],
                      [0, 0, 1])
        # input matrix
        fu = np.array([dt * np.cos(yaw), 0],
                      [dt * np.sin(yaw), 0],
                      [0, 1])

        # perform a prediction
        state_est = np.array([x + dt*lin_vel*np.cos(yaw)],
                             [y + dt*lin_vel*np.sin(yaw)],
                             [yaw + dt*ang_vel])
        state_noise = fx*state_noise*fx.transpose() + fu*vel_noise*fu.transpose()+ mod_noise

        # HAVE TO RETURN STATE_EST AND STATE_NOISE TO MAIN PROGRAM

    def kalman_update_gps(self, x, y, yaw, gps_x, gps_y, h_gps,state_noise, gps_noise):
        # sensor reading
        z = np.array([[gps_x],[gps_y]])

        # perform an update
        z_est = h_gps*np.array([[x],[y],[yaw]])
        innovation = z-z_est
        s = h_gps*state_noise*h_gps.transpose() + gps_noise
        w = state_noise*h_gps.transpose()*np.linalg.inv(s)

        state_est = np.array([[x],[y],[yaw]]) + w*innovation
        state_noise = state_noise - w*s*w.transpose()

        # HAVE TO RETURN STATE_EST AND STATE_NOISE TO MAIN PROGRAM

    def kalman_update_imu(self, x, y, yaw, imu_yaw, h_imu,state_noise, imu_noise):
        # sensor reading
        z = np.array([imu_yaw])

        # NEED TO MAKE SURE THAT THE ANGLE READINGS ARE CORRECT I.E CIRCULAR

        # perform an update
        z_est = h_imu * np.array([[x], [y], [yaw]])
        innovation = z - z_est
        s = h_imu * state_noise * h_imu.transpose() + imu_noise
        w = state_noise * h_imu.transpose() * np.linalg.inv(s)

        state_est = np.array([[x], [y], [yaw]]) + w * innovation
        state_noise = state_noise - w * s * w.transpose()

        # HAVE TO RETURN STATE_EST AND STATE_NOISE TO MAIN PROGRAM

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
