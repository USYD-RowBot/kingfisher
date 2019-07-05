import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs import Float32

def imu_callback(data):

class Node():
    def __init__(self):
        self.current_heading = 0
        self.left = 0
        self.right = 0
        self.desired_heading = 0 #Radianns

    def imu_callback(self,imu_msg):
        (roll, pitch, yaw) = euler_from_quaternion (imu_msg.orientation)
        self.current_heading = yaw

    def heading_callback(self,data):
        self.desired_heading = math.radians(data.data)

if __name__=="__main__":
    rospy.init_node("heading_holder", anonymous=True)
    node = Node()
    rospy.Subscriber("imu",Imu, node.imu_callback)
    pub_left = rospy.Publisher("left", Float32, queue_size = 10)
    pub_right = rospy.Publisher("right", Float32, queue_size = 10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown:
        K = 50;
        err = node.current_heading - node.desired_heading

        node.left = -err*K
        node.right = err*K

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
