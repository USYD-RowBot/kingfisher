#!/usr/bin/env python
from sensor_msgs.msg import NavSatFix
from parse import *
import rospy
import navio.util
import navio.ublox

from std_msgs.msg import Header

# Initialise ros
rospy.init_node('gps',anonymous=True)
OUTPUT_TOPIC = rospy.get_param("~topic",'GPS')
publisher=rospy.Publisher(OUTPUT_TOPIC,NavSatFix,queue_size=10)

# stuff i copied from navio code
ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

ubl.configure_poll_port()
ubl.configure_poll(navio.ublox.CLASS_CFG, navio.ublox.MSG_CFG_USB)
#ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)

ubl.configure_port(port=navio.ublox.PORT_SERIAL1, inMask=1, outMask=0)
ubl.configure_port(port=navio.ublox.PORT_USB, inMask=1, outMask=1)
ubl.configure_port(port=navio.ublox.PORT_SERIAL2, inMask=1, outMask=0)
ubl.configure_poll_port()
ubl.configure_poll_port(navio.ublox.PORT_SERIAL1)
ubl.configure_poll_port(navio.ublox.PORT_SERIAL2)
ubl.configure_poll_port(navio.ublox.PORT_USB)
ubl.configure_solution_rate(rate_ms=1000)

ubl.set_preferred_dynamic_model(None)
ubl.set_preferred_usePPP(None)

ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSLLH, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_PVT, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_STATUS, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SOL, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELNED, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SVINFO, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELECEF, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSECEF, 1)
ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_RAW, 1)
ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SFRB, 1)
ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SVSI, 1)
ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_ALM, 1)
ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_EPH, 1)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_TIMEGPS, 5)
ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_CLOCK, 5)

while not rospy.is_shutdown():
    msg = ubl.receive_message()
    if msg is None:
        pass
    if msg.name() == "NAV_POSLLH":
        outstr = str(msg).split(",")[1:]
        outstr = "".join(outstr)
        components=search(" Longitude={:d} Latitude={:d} Height={:d}",outstr)
        h=Header()
        h.stamp=rospy.Time.now()
        navmsg = NavSatFix()
        navmsg.header=h
        navmsg.latitude=float(components[1])/10000000.0
        navmsg.longitude=float(components[0])/10000000.0
        navmsg.altitude=float(components[2])/10000000.0
        publisher.publish(navmsg)
