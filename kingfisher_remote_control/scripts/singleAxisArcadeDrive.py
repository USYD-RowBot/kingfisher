#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time
import math
rospy.init_node('singleAxisArcadeDrive', anonymous=True)
XAxisTopic=rospy.get_param("~axisX","axisX")
YAxisTopic=rospy.get_param("~axisY","axisY")
XAxisMin=rospy.get_param("~axisXMin",0)
YAxisMin=rospy.get_param("~axisYMin",0)
XAxisMax=rospy.get_param("~axisXMax",100)
YAxisMax=rospy.get_param("~axisYMax",100)
XAxisCenter=rospy.get_param("~axisXCtr",(XAxisMin+XAxisMax)/2) 
YAxisCenter=rospy.get_param("~axisYCtr",YAxisMin)# no reversing if this is unset
maxPower=rospy.get_param("~maxPower",100)
minPower=rospy.get_param("~minPower",0)
centerPower=rospy.get_param("~ctrPower",minPower)# if this is unset, no reversing also
leftMotorChannel=rospy.get_param("~LeftMotorChannel","left_motor")
rightMotorChannel=rospy.get_param("~RightMotorChannel","right_motor")
deadTimeout=0.1#s - probably reduce this to something less generous :/
lastXTime=0
lastYTime=0 # if either signal times out, cut the power.
Xfactor=0
Yfactor=0
rigourousDebug=False
def Xcallback(data):
    global Xfactor
    global lastXTime
    _Xfactor=float(data.data)
    #map from rect to unit square
    if _Xfactor>XAxisCenter:
        Xfactor = (_Xfactor-XAxisCenter)/(XAxisMax-XAxisCenter)
    elif  XAxisCenter != XAxisMin:
        Xfactor = -(XAxisCenter-_Xfactor)/(XAxisCenter-XAxisMin)
    else:
        Xfactor=0
    lastXTime=time.time()
    #if (rigourousDebug):print("X:{0}@{1}".format(Xfactor,lastXTime))

def Ycallback(data):
    global Yfactor
    global lastYTime
    _Yfactor=float(data.data)
    #map from rect to unit square
    if _Yfactor>YAxisCenter:
        Yfactor = (_Yfactor-YAxisCenter)/(YAxisMax-YAxisCenter)
    elif (YAxisCenter!=YAxisMin):
        Yfactor = -(YAxisCenter-_Yfactor)/(YAxisCenter-YAxisMin)
    else:
        Yfactor=0
    lastYTime=time.time()
    #if (rigourousDebug):print("Y:{0}@{1}".format(Yfactor,lastYTime))



# left and right subscribers
rospy.Subscriber(XAxisTopic,Float32,Xcallback)
rospy.Subscriber(YAxisTopic,Float32,Ycallback)
# left and right publishers
pub=[rospy.Publisher(leftMotorChannel, Float32, queue_size=10),rospy.Publisher(rightMotorChannel, Float32, queue_size=10)]
print("Mapping {0},{1} ==> {2},{3}".format(XAxisTopic,YAxisTopic,leftMotorChannel,rightMotorChannel))
while not rospy.is_shutdown():
    #convert into radial coords
    rXY=(Xfactor**2+Yfactor**2)**0.5
    thXY=math.atan2(Yfactor,Xfactor)
    thXY=thXY+math.pi*0.25
    rXY=min(1,rXY)
    #convert into left/right square coords
    sqL=rXY*math.sin(thXY)
    sqR=rXY*math.cos(thXY)
    #convert into L/R thrust vector
    if (sqL>0): outL=centerPower+sqL*(maxPower-centerPower)
    else: outL=centerPower+sqL*(centerPower-minPower)
    if (sqR>0):outR=centerPower+sqR*(maxPower-centerPower)
    else: outR=centerPower+sqR*(centerPower-minPower)
    if (rigourousDebug):print("X:{0:0.00} Y:{1:0.00} rXY:{2:0.00} thXY:{3:0.00} sqL:{4:0.00} sqR:{5:0.00} outL:{6:0.00} outR:{7:0.00}".format(float(Xfactor),float(Yfactor),float(rXY),float(thXY),float(sqL),float(sqR),float(outL),float(outR)))
    if time.time()-lastXTime<deadTimeout and time.time()-lastYTime<deadTimeout:
        pub[0].publish(outL)
        pub[1].publish(outR)
