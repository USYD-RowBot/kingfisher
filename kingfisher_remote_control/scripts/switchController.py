#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32,Bool
import time
rospy.init_node('switchController', anonymous=True)
# Toggle between manual to estop to auto
# Optional primer which fires when auto is started
manualLeft=rospy.get_param("~manualLeft","left")
manualRight=rospy.get_param("~manualRight","right")

autoLeft=rospy.get_param("~autoLeft","left")
autoRight=rospy.get_param("~autoRight","right")

motorLeft=rospy.get_param("~motorLeft","left")
motorRight=rospy.get_param("~motorRight","right")

switchChannel=rospy.get_param("~switchChannel",True)

prime=rospy.get_param("~prime",True)
primerChannel=rospy.get_param("~primeChannel","prime")

manual_state=0
auto_state=1
estop_state=-1

state=estop_state

def manualMotorleftCallback(data):
    global state
    global leftpub
    if state==manual_state:
        leftpub.publish(data.data)

def manualMotorrightCallback(data):
    global state
    global rightpub
    if state==manual_state:
        rightpub.publish(data.data)

def autoMotorleftCallback(data):
    global state
    global leftpub
    if state==auto_state:
        leftpub.publish(data.data)

def autoMotorrightCallback(data):
    global state
    global rightpub
    if state==auto_state:
        rightpub.publish(data.data)

def switchCallback(data):
    global state, manual_state, auto_state,estop_state
    global prime, primepub
    if data.data>0.9:
        if state != auto_state:
            print("primer toggle")
            if prime:
                print ("primer set!")
                primepub.publish(True)
        state=auto_state
    elif data.data<-0.9:
        state=estop_state
    else:
        state=manual_state

# left and right subscribers
rospy.Subscriber(manualLeft,Float32,manualMotorleftCallback,queue_size=10)
rospy.Subscriber(manualRight,Float32,manualMotorrightCallback,queue_size=10)
rospy.Subscriber(autoLeft,Float32,autoMotorleftCallback,queue_size=10)
rospy.Subscriber(autoRight,Float32,autoMotorrightCallback,queue_size=10)

rospy.Subscriber(switchChannel,Float32,switchCallback)

leftpub=rospy.Publisher(motorLeft, Float32, queue_size=10)
rightpub=rospy.Publisher(motorRight, Float32, queue_size=10)

#primer publisher
primepub=rospy.Publisher(primerChannel, Bool, queue_size=10)

#remote reciever


rospy.spin()
    
    
