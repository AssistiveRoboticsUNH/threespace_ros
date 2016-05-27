#!/usr/bin/env python
import roslib
import rospy
import string
import find_dng
import threespace_api as tsa
from threespace_api import *

rospy.init_node("pairer")

dng_list = find_dng.returnDev("dng")
dev_list = find_dng.returnDev("dev")
if(dng_list==[])or(dev_list==[]):
    exit()
rospy.logwarn(dng_list)
rospy.logwarn(dev_list)
d = dng_list[0]
wc = None
att=0
while wc == None:
    if att > 10:
        exit()
    att+=1
    try:
        rospy.logwarn("Getting Wireless Channel")
        wc = tsa.TSDongle.getWirelessChannel(d)
    except:
        pass
rospy.logerr(wc)
rospy.logerr(d.wireless_table)
for dev in dev_list:
    rospy.logwarn("Getting Sensor ID")
    #rospy.logwarn(convertString(dev.f7WriteRead('getSerialNumber')))
    hw_id = dev.serial_number
    rospy.logwarn("%s",hw_id)
    if hw_id in d.wireless_table:
        rospy.loginfo("Device already registered with dongle")
    else:
        rospy.logwarn(tsa.TSWLSensor.getWirelessChannel(dev))
        tsa.TSWLSensor.setWirelessPanID(dev, int(wc))
        tsa.TSWLSensor.setWirelessChannel(dev, int(wc))
        rospy.logerr(tsa.TSWLSensor.getWirelessChannel(dev))
        for i in range(0,15):
            if(d.wireless_table[i]<100):
                rospy.loginfo("Found free spot at [%d]=%d",i ,d.wireless_table[i])
                tsa.TSDonlge.setSensorToDongle(d, i, hw_id)
tsa.TSDongle.commitWirelessSettings(d)
rospy.logwarn(d.wireless_table)
