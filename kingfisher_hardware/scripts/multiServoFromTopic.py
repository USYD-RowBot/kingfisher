#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Float32
import sys
import time
import navio.pwm

num_pwms=13
rospy.init_node("servoNode",anonymous=True)
PWM_TOPIC=[rospy.get_param("~servoTopic{0}".format(i),"servo{0}".format(i)) for i in range(num_pwms)]
PWM_ENABLED=[rospy.has_param("~servoTopic{0}".format(i)) for i in range(num_pwms)]
[print("initialised topic {0} for {1}".format(PWM_TOPIC[i],i)) if PWM_ENABLED[i] else None for i in range(num_pwms)]
pwm=[navio.pwm.PWM(i) if PWM_ENABLED[i] else None for i in range(num_pwms)]

def callback(pwm,val):
    pwm.set_duty_cycle(val);

def startPWM(i):
    global pwm
    global PWM_TOPIC
    if pwm[i] is None:
        return
    pwm[i].initialize()
    pwm[i].set_period(50);
    pwm[i].enable()
    rospy.Subscriber(PWM_TOPIC[i],Float32,lambda data: callback(pwm[i],data.data))

[startPWM(i) for i in range(num_pwms)]
rospy.spin()
[pwm[i].deinitialize() if PWM_ENABLED[i] else None for i in range(num_pwms)]
