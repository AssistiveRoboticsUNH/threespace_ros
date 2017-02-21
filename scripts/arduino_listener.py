#!/usr/bin/env python
import time
import sys
import rospy
# import config
import serial
import numpy as np
from gait_hmm_ros.msg import ardu_msg
from xbee import XBee
from xbee import ZigBee


data = ardu_msg()


def make_msg(frame):
    try:	
        # data = frame
	# data = ardu_msg()
        fr_data = (frame['rf_data'][1:-1]).split(",")
        #print fr_data[0]
        data.header.frame_id = 'arduino'
        data.ir = int(fr_data[0]) if (int(fr_data[0])<20000) else data.ir
        data.prox = float(fr_data[1])
        data.fsrfl = int(fr_data[2])
        data.fsrfr = int(fr_data[4])
        data.fsrbk = int(fr_data[3])
	# print data.fsrbk
        print frame['rf_data']
    except:
        pass


# PORT = rospy.get_param('~xbee_port', '/dev/ttyUSB0')
PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

ser = serial.Serial(PORT, BAUD_RATE)

rospy.init_node('arduino_listener')

# for requested input
# dev = XBee(ser, escaped=True)
# dev = ZigBee(ser, escaped=True)

# for asynchronous use
# dev = XBee(ser, callback=make_msg)
dev = ZigBee(ser, callback=make_msg)

arduPub = rospy.Publisher('arduino', ardu_msg, queue_size=1)
r = rospy.Rate(10)

while not rospy.is_shutdown():
	try:
	    # inc = dev.wait_read_frame()
	    # make_msg(frame)
            # print('-----------')	
	    data.header.stamp = rospy.Time.now()
	    arduPub.publish(data)
	    r.sleep()
	except:
	    pass

#         if message_received == 1:
#             arduPub.publish(data)
#             r.sleep()

#while not rospy.is_shutdown():
#while True:
#    try:
#        make_msg(dev.wait_read_frame())
#        data.header.stamp = rospy.Time.now()
#        arduPub.publish(data)
#    except :
#        rospy.logerr("timeout")

dev.halt()
ser.close()
