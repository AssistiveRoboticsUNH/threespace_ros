#!/usr/bin/env python
import roslib
import rospy
import serial
import time
import math
import string
import glob
import sensor_table
import find_dng
import threespace as tsa
from threespace import *
from socket import *

rospy.init_node("charge_monitor")
r = rospy.Rate(100)

devlist = find_dng.returnDev("dev")
if len(devlist)==0 :
    rospy.logwarn("No devices found, exiting")
    exit()
    
while not rospy.is_shutdown():
    for device in devlist:
        id_ = str(device)
        id_ = id_[id_.find('W'):-1]
        rospy.logwarn("Device : %s is at %d", id_, tsa.TSWLSensor.getBatteryPercentRemaining(device))
    rospy.sleep(rospy.Duration(5))
    rospy.logerr("------------------------")

for d in devlist:
    tsa.TSWLSensor.close(d)
