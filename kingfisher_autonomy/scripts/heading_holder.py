#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
import math


class Node():
    def __init__(self):
        self.current_heading = 0
        self.left = 0
        self.right = 0
        self.desired_heading = 0 #Radianns
        self.yaw_rate = 0

    def imu_callback(self,imu_msg):
        (roll, pitch, yaw) = euler_from_quaternion ([imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w])
        self.current_heading = yaw
        print("YAW:", math.degrees(yaw))
        self.yaw_rate = imu_msg.angular_velocity.z

    def heading_callback(self,data):
        self.desired_heading = math.radians(data.data)
        print("recieved deasired", self.desired_heading)

if __name__=="__main__":
    rospy.init_node("heading_holder", anonymous=True)
    node = Node()
    rospy.Subscriber("imu",Imu, node.imu_callback)
    rospy.Subscriber("heading",Float32,node.heading_callback)
    pub_left = rospy.Publisher("left", Float32, queue_size = 10)
    pub_right = rospy.Publisher("right", Float32, queue_size = 10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        Kp = 500
        Kd = 50
        err = node.current_heading - node.desired_heading
        #dt = node.current_heading - previous_heading
        node.left = err*K + node.yaw_rate * Kd
        node.right = -err*K - node.yaw_rate*Kd


        if (node.left>100):
            node.left = 100
        if (node.left<-100):
            node.left = -100
        if (node.right>100):
            node.right = 100
        if (node.right<-100):
            node.right = -100

        pub_left.publish(node.left)
        pub_right.publish(node.right)
        rate.sleep()
