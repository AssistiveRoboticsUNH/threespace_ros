import serial
import time
import math
import string
import rospy
import threespace_api as tsa
import find_ports
from threespace_api import *
from socket import *

def returnDev(arg):
    #rospy.init_node("detect")
    result = find_ports.findPorts()
    dng_list = []
    dev_list = []
    rospy.logwarn(result)
    if(arg=="dng"):
        for a_port in result:
            try:
                rospy.loginfo("Checking for dongle in port %s", a_port)
                dongle = tsa.TSDongle(com_port = a_port, baudrate=115200)
                rospy.logwarn(dongle)
                hwid = convertString(dongle.f7WriteRead('getSerialNumber'))
                panid = None
                channel = None
                wa = None
                att = 0
                while((panid==None)or(channel==None)or(wa==None)):
                    try:
                        att+=1
                        rospy.loginfo("Attempting to get Dongle data, attempt #%d",att)
                        paid = tsa.TSDongle.getWirelessPanID(dongle)
                        channel  = tsa.TSDongle.getWirelessChannel(dongle)
                        wa = tsa.TSDongle.getWirelessAddress(dongle)
                        rospy.logwarn("HW ID : %s", hwid)
                        rospy.logwarn("PanID : %s", panid)
                        rospy.logwarn("Channel : %s", channel)
                        rospy.logwarn("Wireless Address %s", wa)
                        rospy.logwarn(dongle.wireless_table)
                        dng_list.append(dongle)
                        if(att>10):
                            break
                        break
                    except:
                        pass
            except:
                pass
        print dng_list
        return dng_list
    else:
        for a_port in result:
            try:
                rospy.loginfo("Checking for dongle in port %s", a_port)
                sensor = tsa.TSWLSensor(com_port = a_port, baudrate=115200)
                rospy.logwarn(dongle)
                hwid = convertString(dongle.f7WriteRead('getSerialNumber'))
                panid = None
                channel = None
                wa = None
                att = 0
                while((panid==None)or(channel==None)or(wa==None)):
                    try:
                        att+=1
                        rospy.loginfo("Attempting to get Wireless Sensor data, attempt $%d",att)
                        paid = tsa.TSDongle.getWirelessPanID(sensor)
                        channel  = tsa.TSDongle.getWirelessChannel(sensor)
                        wa = tsa.TSDongle.getWirelessAddress(sensor)
                        rospy.logwarn("HW ID : %s", hwid)
                        rospy.logwarn("PanID : %s", panid)
                        rospy.logwarn("Channel : %s", channel)
                        rospy.logwarn("Wireless Address %s", wa)
                        rospy.logwarn(sensor.wireless_table)
                        dev_list.append(sensor)
                        if(att>10):
                            break
                        break
                    except:
                        pass
            except:
                pass
        print dev_list
        return dev_list

