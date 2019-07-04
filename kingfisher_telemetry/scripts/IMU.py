#!/usr/bin/env python
"""
MS5611 driver code is placed under the BSD license.
Copyright (c) 2014, Emlid Limited, www.emlid.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import spidev
import time
import sys
import navio.mpu9250
import navio.util
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

navio.util.check_apm()

rospy.init_node("imusuite", anonymous=True)

INSTRUMENT = rospy.get_param("~instrument", "mpu")  # or lsm - but lsm is not implemented
IMU_CHANNEL = rospy.get_param("~imuChannel", "imu")
MAG_CHANNEL = rospy.get_param("~magChannel", "mag")

if INSTRUMENT == 'mpu':
    print "Selected: MPU9250"
    imu = navio.mpu9250.MPU9250()
elif INSTRUMENT == 'lsm':
    print "Selected: LSM9DS1"
    imu = navio.lsm9ds1.LSM9DS1()
else:
    print "Wrong sensor name. Select: mpu or lsm"
    sys.exit(1)

if imu.testConnection():
    print "Connection established: True"
else:
    sys.exit("Connection established: False")

imu.initialize()

rospy.sleep(1)
rate = rospy.Rate(10)
imu_pub = rospy.Publisher(IMU_CHANNEL, Imu)
mag_pub = rospy.Publisher(MAG_CHANNEL, MagneticField)
while not rospy.is_shutdown():
    # imu.read_all()
    # imu.read_gyro()
    # imu.read_acc()
    # imu.read_temp()
    # imu.read_mag()

    # print "Accelerometer: ", imu.accelerometer_data
    # print "Gyroscope:     ", imu.gyroscope_data
    # print "Temperature:   ", imu.temperature
    # print "Magnetometer:  ", imu.magnetometer_data
    # time.sleep(0.1)
    h=Header()
    h.stamp=rospy.Time.now()
    m9a, m9g, m9m = imu.getMotion9()
    imu_msg = Imu()
    imu_msg.header=h
    imu_msg.angular_velocity.x=m9g[0]
    imu_msg.angular_velocity.y=m9g[1]
    imu_msg.angular_velocity.z=m9g[2]
    imu_msg.linear_acceleration.x=m9a[0]
    imu_msg.linear_acceleration.y=m9a[1]
    imu_msg.linear_acceleration.z=m9a[2]
    imu_pub.publish(imu_msg)

    mag_msg= MagneticField()
    mag_msg.header=h
    mag_msg.magnetic_field.x=m9m[0]
    mag_msg.magnetic_field.y=m9m[1]
    mag_msg.magnetic_field.z=m9m[2]
    mag_pub.publish(mag_msg)
    
    rate.sleep()
