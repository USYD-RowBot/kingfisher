#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32
from std_msgs.msg import Header
rospy.init_node("compass",anonymous=True)
MAG_IN_TOPIC=rospy.get_param("magTopic","mag")
COMPASS_TOPIC=rospy.get_param("compassTopic","compass")
pub=rospy.Publisher(COMPASS_TOPIC,Float32)
def cb(data):
    Xfield=data.magnetic_field.x
    Yfield=data.magnetic_field.y
    bearing=math.atan2(Yfield,Xfield)*180.0/3.141
    print (bearing)
    pub.publish(bearing)

sub=rospy.Subscriber(MAG_IN_TOPIC,MagneticField,cb)
rospy.spin()
