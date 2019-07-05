#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import sys
import time

rospy.init_node("multiServoClient",anonymous=True)
PWM_OUTPUT=rospy.get_param("~srv_num",0)
PWM_MAX=rospy.get_param("~val_max",100.0)
PWM_MIN=rospy.get_param("~val_min",0.0)
TIME_MAX=rospy.get_param("~time_max",2.00)# in ms
TIME_MIN=rospy.get_param("~time_min",1.00)
PRIME_REQUIRED=rospy.get_param("~prime",False)
topic=rospy.get_param("~topic","thrust")
topicOut=rospy.get_param("~topicOut","servoTopic{0}".format(PWM_OUTPUT))
print(rospy.get_param("~topic"))
# don't actually use navio pwm -  just forward commands

pub = rospy.Publisher(topicOut, Float32, queue_size=10)



def pwmForward (val):
    pub.publish(val);

if (PRIME_REQUIRED):
    # perform a priming sequence, by setting to zero, 0.5, 0 for some time.
    
    rate=rospy.Rate(1000)
    for i in range(2000):
        ((TIME_MAX+TIME_MIN)/2.0)
        rate.sleep()

    for i in range(2000):
        pwmForward((TIME_MAX*1.5+TIME_MIN*0.5)/2.0)
        rate.sleep()

    for i in range(2000):
        pwmForward((TIME_MAX+TIME_MIN)/2.0)
        rate.sleep()
    #priming done!

def callback(data):
    scaled=data.data;
    # clip the range
    if (scaled>PWM_MAX):
        scaled=PWM_MAX
    if (scaled<PWM_MIN):
        scaled=PWM_MIN
    # get the corresponding value
    scaled=(1.0*scaled-1.0*PWM_MIN)/(1.0*PWM_MAX-1.0*PWM_MIN)*(TIME_MAX-TIME_MIN)+TIME_MIN;
    pwmForward(scaled)
    # set the servo power
rospy.Subscriber(topic,Float32,callback)
rospy.spin()
