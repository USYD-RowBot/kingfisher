#!/usr/bin/env python
from __future__ import print_function
# so i can use it with my list comprehensions
import rospy
from std_msgs.msg import Float32
import sys
import time
import serial


## Following code is modified code based on code from samfok on github:
## https://github.com/samfok/remote_receiver_tutorial/blob/master/main.py
def align_serial(ser):
    """Aligns the serial stream with the incoming Spektrum packets
    Spektrum Remote Receivers (AKA Spektrum Satellite) communicate serially
    in 16 byte packets at 125000 bits per second (bps)(aka baud) but are
    compatible with the standard 115200bps rate. We don't control the output
    transmission timing of the Spektrum receiver unit and so might start
    reading from the serial port in the middle of a packet transmission.
    To align the reading from the serial port with the packet transmission,
    we use the timing between packets to detect the interval between packets
    Packets are communicated every 11ms. At 115200 bps, a bit is read in 
    approximately 8.69us, so a 16 byte (128 bit)
    packet will take around 1.11ms to be communicated, leaving a gap of about
    9.89ms between packets. We align our serial port reading with the protocol
    by detecting this gap between reads.
    Note that we do not use the packet header contents because
        1) They are product dependent. Specifically, "internal" Spektrum
        receivers indicate the system protocol in the second byte of the header
        but "external" receivers do not. Further, different products are
        use different protocols and indicate this using the
        system protocol byte.
        2) Other bytes in the packet may take on the same value as the header
        contents. No bit patterns of a byte are reserved, so any byte in the
        data payload of the packet could match the values of the header bytes.
    Inputs
    ------
    ser: serial.Serial instance
        serial port to read from
    """
    # read in the first byte, might be a long delay in case the transmitter is
    # off when the program begins
    while ser.inWaiting()==0:
        if rospy.is_shutdown():
           return
        time.sleep(0.01)
    ser.read(1)
    dt = 0
    # wait for the next long delay between reads
    dt_threshold = 0.005 # pick some threshold between 8.69us and 9.89ms
    while dt < dt_threshold and not rospy.is_shutdown():
        start = time.time()
        ser.read()
        dt = time.time()-start
    # consume the rest of the packet
    ser.read(15)
    # should be aligned with protocol now

MASK_CH_ID = 0b01111000 # 0x78
SHIFT_CH_ID = 3
MASK_SERVO_POS_HIGH = 0b00000111 # 0x07
data = None
def parse_channel_data(data):
    """Parse a channel's 2 bytes of data in a remote receiver packet
    Inputs
    ------
    data: 2 byte long string (currently only supporting Python 2)
        Bytes within the remote receiver packet representing a channel's data
    Outputs
    -------
    channel_id, channel_data
    """
    ch_id = (ord(data[0]) & MASK_CH_ID) >> SHIFT_CH_ID
    ch_data = (
        ((ord(data[0]) & MASK_SERVO_POS_HIGH) << 8) | ord(data[1]))
    ch_data = 988 + (ch_data >> 1)
    return ch_id, ch_data






rospy.init_node('remote_control', anonymous=True)

PORT_NAME=rospy.get_param("~port",'/dev/ttyUSB0')
CHANNEL_DEFAULT_MAX=[1840,1840,1840,1840,1,1]
CHANNEL_DEFAULT_MIN=[1159,1159,1159,1159,0,0]
# get parameters for channels and remapping from ROS
TOPIC_OUT=[rospy.get_param("~topic_out{0}".format(i),"remote_out_{0}".format(i)) for i in range(6)]
ENABLED=[rospy.has_param("~topic_out{0}".format(i)) for i in range(6)]
CHANNEL_MAX=[rospy.get_param("~channel_max{0}".format(i),CHANNEL_DEFAULT_MAX[i]) for i in range(6)]
CHANNEL_MIN=[rospy.get_param("~channel_min{0}".format(i),CHANNEL_DEFAULT_MIN[i]) for i in range(6)]
CHANNEL_OUT_MAX=[rospy.get_param("~channel_out_max{0}".format(i),CHANNEL_MAX[i]) for i in range(6)]
CHANNEL_OUT_MIN=[rospy.get_param("~channel_out_min{0}".format(i),CHANNEL_MIN[i]) for i in range(6)]
#[print(rospy.get_param("topic_out{0}".format(i))) for i in range(6)]
#print (rospy.get_param(")
N_CHAN = 13
data = None
servo_position = [0 for i in range(N_CHAN)]
print("Initialisation: port {0}".format(PORT_NAME))

[print("initialised topic {0} for {1}".format(TOPIC_OUT[i],i)) if ENABLED[i] else None for i in range(6)]
ser = serial.Serial(
    port=PORT_NAME, baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE)

#start ROS
publishers = [rospy.Publisher(TOPIC_OUT[i], Float32, queue_size=10) if ENABLED[i] else None for i in range(6)]
#start the serial
try:
    align_serial(ser)
    while not rospy.is_shutdown():
        data_buf = ser.read(16)
        data = data_buf[2:]
        for i in range(7):
            ch_id, s_pos = parse_channel_data(data[2*i:2*i+2])
            servo_position[ch_id] = s_pos

        # remap all variables

        mapped_servo_position = [CHANNEL_OUT_MIN[i]+(CHANNEL_OUT_MAX[i]-CHANNEL_OUT_MIN[i])*(servo_position[i]-CHANNEL_MIN[i])/(CHANNEL_MAX[i]-CHANNEL_MIN[i]) for i in range(6)];

        # publish to all channels
        [publishers[i].publish(mapped_servo_position[i]) if ENABLED[i]==True else None for i in range(6)]

        #? echo a 'received' to the serial? perhaps? not sure
        ser.write(data_buf)
except(KeyboardInterrupt, SystemExit):
    ser.close()
except(Exception) as ex:
    print (ex)
    ser.close()
print("ok im done")
#shutdown
ser.close()
sys.exit()
