#!/usr/bin/env python
import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import serial
import time
import math
import string
import glob
import sensor_table
import find_dng
import threespace_api as tsa
from threespace_api import *
from socket import *

rospy.init_node("publisher")
r = rospy.Rate(100)
dongle_list = find_dng.returnDev("dng")
publishers = {}
broadcasters = {}
for d in dongle_list:
    tsa.TSDongle.broadcastSynchronizationPulse(d)
    for device in d:
        if device == None:
            rospy.logerr("No Sensor Found")
        else:
            quat = tsa.TSWLSensor.getTaredOrientationAsQuaternion(device)
            id = str(device)
            id = id[id.find('W'):-1]
            rospy.logerr(id)
            #rospy.logerr(quat)
            frame = sensor_table.sensor_table.get(id)
            rospy.logerr("Adding publisher for %s : %s",id,frame)
            br = tf2_ros.TransformBroadcaster()
            pub = rospy.Publisher(frame, geometry_msgs.msg.QuaternionStamped, queue_size = 10)
            broadcasters[frame] = br
            publishers[frame] = pub

t = geometry_msgs.msg.TransformStamped()
g = geometry_msgs.msg.QuaternionStamped()
while not rospy.is_shutdown():
    #r.sleep()
    for d in dongle_list:
        for device in d:
            if device is not None:
                quat = tsa.TSWLSensor.getTaredOrientationAsQuaternion(device)
                if quat is not None:
                    id = str(device)
                    id = id[id.find('W'):-1]
                    frame = sensor_table.sensor_table.get(id)
                    #rospy.logwarn("%s : %s ---> %s",id,frame,quat)
                    p = publishers.get(frame)
                    b = broadcasters.get(frame)
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "world"
                    t.child_frame_id = frame
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = 0.0
                    g.quaternion.x = quat[0]
                    g.quaternion.y = quat[1]
                    g.quaternion.z = quat[2]
                    g.quaternion.w = quat[3]
                    t.transform.rotation = g.quaternion
                    b.sendTransform(t)
                    p.publish(g)

print publishers
print broadcasters
t = geometry_msgs.msig.TransformStamped()
exit()
#brs = tf2_ros.StaticTransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()
gmsg = geometry_msgs.msg.QuaternionStamped()
pub = rospy.Publisher('head', geometry_msgs.msg.QuaternionStamped, queue_size=1)
id = tsa.TSWLSensor.getWirelessPanID(device)
#id = tsa.TSWLSensor.get
print id
while not rospy.is_shutdown():
    quat = tsa.TSWLSensor.getTaredOrientationAsQuaternion(device)
    #rospy.loginfo(quat)
    gmsg.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "head"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    gmsg.quaternion.x = quat[0]
    gmsg.quaternion.y = quat[1]
    gmsg.quaternion.z = quat[2]
    gmsg.quaternion.w = quat[3]
    t.transform.rotation = gmsg.quaternion
    br.sendTransform(t)
    pub.publish(gmsg)
    r.sleep()

