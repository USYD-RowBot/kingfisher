#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import time
import math
rospy.init_node('pointrecord', anonymous=True)

GPSChannel=rospy.get_param("~GPSChannel","GPS")
compassChannel=rospy.get_param("~compassChannel","compass")

maxSpeed=rospy.get_param("~maxSpeed",50)

outputLeft=rospy.get_param("~outputLeft","left")
outputRight=rospy.get_param("~outputRight","right")

switchChannel=rospy.get_param("~switchChannel",True)

recordedLat=0
recordedLon=0

currentLat=0
currentLon=0

def GPSCallback(data):
    global recordedLat
    global recordedLon
    global currentLat
    global currentLon
    global recordNow
    if recordNow==True:
        recordedLat=data.latitude
        recordedLon=data.longitude
        recordNow=False
    currentLat=data.latitude
    currentLon=data.longitude
currentBearing=None

def compassCallback(data):
    global currentBearing
    currentBearing=data.data

def switchCallback(data):
    global recordNow
    global prestate
    if not data.data==prestate:
       prestate=data.data
       recordNow=True

# left and right subscribers
rospy.Subscriber(switchChannel,Float32,switchCallback)
rospy.Subscriber(GPSChannel,NavSatFix,GPSCallback)
rospy.Subscriber(compassChannel,Compass,compassCallback)

leftpub=rospy.Publisher(outputLeft, Float32, queue_size=10)
rightpub=rospy.Publisher(outputRight, Float32, queue_size=10)

#remote reciever

while not rospy.is_shutdown():
    if not currentBearing is None and currentLat !=0 && currentLon != 0 &
     recordedLat !=0 && recordedLon!=0:
       deviationY = recordedLat - currentLat
       deviationX = recordedLon - currentLon
       targetAngle = math.atan2(deviationY, deviationX)-currentBearing
       forward=(math.pi-math.abs(targetAngle))/math.pi*maxspeed
       delta = targetAngle/math.pi*maxspeed
       leftpub.publish(forward + delta)
       rightpub.publish(forward - delta)
