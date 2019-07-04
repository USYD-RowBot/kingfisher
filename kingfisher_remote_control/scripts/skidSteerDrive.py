#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time
import math
rospy.init_node('skidSteerDrive', anonymous=True)
Axis1Topic=rospy.get_param("~axis1","axis1")
Axis2Topic=rospy.get_param("~axis2","axis2")
Axis1Min=rospy.get_param("~axis1Min",0)
Axis2Min=rospy.get_param("~axis2Min",0)
Axis1Max=rospy.get_param("~axis1Max",100)
Axis2Max=rospy.get_param("~axis2Max",100)
Axis1Center=rospy.get_param("~axis1Ctr",(Axis1Min+Axis1Max)/2) 
Axis2Center=rospy.get_param("~axis2Ctr",(Axis2Min+Axis2Max)/2) 
maxPower=rospy.get_param("~maxPower",100)
minPower=rospy.get_param("~minPower",0)
centerPower=rospy.get_param("~ctrPower",(minPower+maxPower)/2)
leftMotorChannel=rospy.get_param("~LeftMotorChannel","left_motor")
rightMotorChannel=rospy.get_param("~RightMotorChannel","right_motor")
deadTimeout=0.1#s - probably reduce this to something less generous :/
last1Time=0
last2Time=0 # if either signal times out, cut the power.
factor1=0
factor2=0
rigourousDebug=False
def callback1(data):
    global factor1
    global last1Time
    _1factor=float(data.data)
    #map from rect to unit square
    if _1factor>Axis1Center:
        factor1 = (_1factor-Axis1Center)/(Axis1Max-Axis1Center)
    elif  Axis1Center != Axis1Min:
        factor1 = -(Axis1Center-_1factor)/(Axis1Center-Axis1Min)
    else:
        factor1=0
    last1Time=time.time()
    #if (rigourousDebug):print(":1{0}@{1}".format(factor1,lastTime1))

def callback2(data):
    global factor2
    global last2Time
    _factor2=float(data.data)
    #map from rect to unit square
    if _factor2>Axis2Center:
        factor2 = (_factor2-Axis2Center)/(Axis2Max-Axis2Center)
    elif (Axis2Center!=Axis2Min):
        factor2 = -(Axis2Center-_factor2)/(Axis2Center-Axis2Min)
    else:
        factor2=0
    last2Time=time.time()
    #if (rigourousDebug):print(":2{0}@{1}".format(factor2,lastTime2))



# left and right subscribers
rospy.Subscriber(Axis1Topic,Float32,callback1)
rospy.Subscriber(Axis2Topic,Float32,callback2)
# left and right publishers
pub=[rospy.Publisher(leftMotorChannel, Float32, queue_size=10),rospy.Publisher(rightMotorChannel, Float32, queue_size=10)]
print("Mapping {0},{1} ==> {2},{3}".format(Axis1Topic,Axis2Topic,leftMotorChannel,rightMotorChannel))
while not rospy.is_shutdown():
    #convert into radial coords
    #convert into left/right square coords
    sqL=factor1
    sqR=factor2
    #convert into L/R thrust vector
    if (sqL>0): outL=centerPower+sqL*(maxPower-centerPower)
    else: outL=centerPower+sqL*(centerPower-minPower)
    if (sqR>0):outR=centerPower+sqR*(maxPower-centerPower)
    else: outR=centerPower+sqR*(centerPower-minPower)
    if (rigourousDebug):print(":1{0:0.00} :2{1:0.00} sqL:{2:0.00} sqR:{3:0.00} outL:{4:0.00} outR:{5:0.00}".format(float(factor1),float(factor2),float(sqL),float(sqR),float(outL),float(outR)))
    if time.time()-last1Time<deadTimeout and time.time()-last2Time<deadTimeout:
        pub[0].publish(outL)
        pub[1].publish(outR)
