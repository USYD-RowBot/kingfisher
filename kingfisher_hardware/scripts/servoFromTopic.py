#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import sys
import time
import navio.pwm
#import navio.util

rospy.init_node("servoNode",anonymous=True)
#navio.util.check_apm()
PWM_OUTPUT=rospy.get_param("~srv_num",0)
PWM_MAX=rospy.get_param("~val_max",100.0)
PWM_MIN=rospy.get_param("~val_min",0.0)
TIME_MAX=rospy.get_param("~time_max",2.00)# in ms
TIME_MIN=rospy.get_param("~time_min",1.00)
PRIME_REQUIRED=rospy.get_param("~prime",False)
topic=rospy.get_param("~topic","thrust")
print(rospy.get_param("~topic"))
rospy.sleep(2*PWM_OUTPUT)# to resolve priming errors
with navio.pwm.PWM(PWM_OUTPUT) as pwm:
    pwm.set_period(50)
    pwm.enable()
    if (PRIME_REQUIRED):
        # perform a priming sequence, by setting to zero, 0.5, 0 for some time.
        
        rate=rospy.Rate(50)
        for i in range(100):
            pwm.set_duty_cycle((TIME_MAX+TIME_MIN)/2)
            rate.sleep()

        for i in range(100):
            pwm.set_duty_cycle((TIME_MAX*1.5+TIME_MIN*0.5)/2)
            rate.sleep()

        for i in range(100):
            pwm.set_duty_cycle((TIME_MAX+TIME_MIN)/2)
            rate.sleep()
        #priming done!
    
    def callback(data):
        print ("servo incoming data:{0}".format(data))
        scaled=data.data;
        # clip the range
        if (scaled>PWM_MAX):
            scaled=PWM_MAX
        if (scaled<PWM_MIN):
            scaled=PWM_MIN
        # get the corresponding value
        scaled=(1.0*scaled-1.0*PWM_MIN)/(1.0*PWM_MAX-1.0*PWM_MIN)*(TIME_MAX-TIME_MIN)+TIME_MIN;
        print ("scaled output:{0}".format(scaled))
        pwm.set_duty_cycle(scaled)
        # set the servo power
    rospy.Subscriber(topic,Float32,callback)
    rospy.spin()
