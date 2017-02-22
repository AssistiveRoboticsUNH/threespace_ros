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
import threespace as tsa
from threespace import *
from ThreeJoint import ThreeJoint
from socket import *
from threespace_ros.msg import dataVec


class SinglePublisher:
    def __init__(self):
        rospy.init_node("publisher")
        r = rospy.Rate(100)
        suffix = "_data_vec"
        dongle_list = find_dng.returnDev("dng")
        publishers = {}
        dv_publishers = {}
        broadcasters = {}
        joints = {}
        upper_topic = rospy.get_param('upper', 'none')
        rospy.logerr(upper_topic)
        if upper_topic == 'none':
            rospy.logerr('No topic for upper joint found, exiting')
        else:
            joints[upper_topic] = ThreeJoint('upper', 'world', 0.30)
            lower_topic = rospy.get_param('lower', 'none')
            rospy.logerr(lower_topic)
            if lower_topic == 'none':
                rospy.logerr('No topic for lower joint found')
            else:
                joints[lower_topic] = ThreeJoint('lower', joints.get('upper').name, 0.30)
                hand_topic = rospy.get_param('hand', 'none')
                rospy.logerr(hand_topic)
                if hand_topic == 'none':
                    rospy.logerr('No topic for hand joint found')
                else:
                    joints[hand_topic] = ThreeJoint('hand', joints.get('hand').name, 0.05)

        if len(dongle_list) == 0:
            rospy.logerr("No dongles found, exiting")
            exit()
        for d in dongle_list:
            tsa.TSDongle.broadcastSynchronizationPulse(d)
            for device in d:
                if device is None:
                    rospy.logerr("No Sensor Found")
                else:
                    id_ = str(device)
                    id_ = id_[id_.find('W'):-1]
                    rospy.logerr(id_)
                    frame = sensor_table.sensor_table.get(id_)
                    rospy.logwarn("Adding publisher for %s : %s", id_, frame)
                    rospy.logwarn("Battery at %s Percent ", tsa.TSWLSensor.getBatteryPercentRemaining(device))
                    br = tf2_ros.TransformBroadcaster()
                    dv_pub = rospy.Publisher(frame + suffix, dataVec, queue_size=100)
                    broadcasters[frame] = br
                    dv_publishers[frame] = dv_pub
                    tsa.TSWLSensor.setStreamingSlots(device, slot0='getTaredOrientationAsQuaternion',
                                                     slot1='getAllCorrectedComponentSensorData')

        t = geometry_msgs.msg.TransformStamped()
        t2 = geometry_msgs.msg.TransformStamped()
        t3 = geometry_msgs.msg.TransformStamped()
        dv = dataVec()
        dev_list = []
        for d in dongle_list:
            for dev in d:
                if dev is not None:
                    dev_list.append(dev)

        while not rospy.is_shutdown():
            # r.sleep()
            # for d in dongle_list:
            for device in dev_list:
                if device is not None:
                    batch = tsa.TSWLSensor.getStreamingBatch(device)
                    if batch is not None:
                        # print(batch)
                        frame = sensor_table.sensor_table.get(id_)
                        quat = batch[0:4]
                        full = batch[4:]
                        id_ = str(device)
                        id_ = id_[id_.find('W'):-1]
                        dp = dv_publishers.get(frame)
                        dv.header.stamp = rospy.get_rostime()
                        dv.quat.quaternion.x = -quat[2]
                        dv.quat.quaternion.y = quat[0]
                        dv.quat.quaternion.z = -quat[1]
                        dv.quat.quaternion.w = quat[3]
                        t.transform.rotation = dv.quat.quaternion
                        dv.gyroX = full[0]
                        dv.gyroY = full[1]
                        dv.gyroZ = full[2]
                        dv.accX = full[3]
                        dv.accY = full[4]
                        dv.accZ = full[5]
                        dv.comX = full[6]
                        dv.comY = full[7]
                        dv.comZ = full[8]

                        t.header.stamp = rospy.Time.now()
                        t2.header.stamp = rospy.Time.now()
                        t3.header.stamp = rospy.Time.now()

                        t.header.frame_id = joints.get(frame).parent
                        t2.header.frame_id = joints.get(frame).parent
                        t3.header.frame_id = joints.get(frame).parent

                        t.child_frame_id = joints.get(frame).name
                        t2.child_frame_id = joints.get(frame).name + '2'
                        t3.child_frame_id = joints.get(frame).name + '3'

                        t.transform.rotation = dv.quat.quaternion
                        t2.transform.rotation = dv.quat.quaternion
                        t3.transform.rotation = dv.quat.quaternion

                        (yaw, pitch, roll) = tf.transformations.euler_from_quaternion(
                            [t.transform.rotation.x,
                             t.transform.rotation.y,
                             t.transform.rotation.z,
                             t.transform.rotation.w])

                        # if joints.get(frame).set is False:
                        joints.get(frame).yaw = yaw
                        joints.get(frame).pitch = pitch
                        joints.get(frame).roll = roll
                        joints.get(frame).set = True

                        # joints.get(frame).roll = roll - joints.get(frame).roll
                        # joints.get(frame).pitch = pitch - joints.get(frame).pitch
                        # joints.get(frame).yaw = yaw - joints.get(frame).yaw

                        if joints.get(frame).parent != 'world':
                            rospy.logerr(joints.get(frame).name+" "+joints.get(frame).parent)
                            joints.get(frame).roll = joints.get(frame).roll + joints.get(joints.get(frame).parent).roll
                            joints.get(frame).yaw = joints.get(frame).yaw + joints.get(joints.get(frame).parent).yaw
                            joints.get(frame).pitch = joints.get(frame).pitch + joints.get(
                                joints.get(frame).parent).pitch

                            t2.transform.rotation = t.transform.rotation
                            joints.get(frame).x = joints.get(frame).radius * math.sin(joints.get(frame).pitch) * math.sin(
                                joints.get(frame).yaw)
                            joints.get(frame).y = joints.get(frame).radius * math.sin(joints.get(frame).pitch) * math.cos(
                                joints.get(frame).yaw)
                            joints.get(frame).z = joints.get(frame).radius * math.cos(joints.get(frame).pitch)

                            # rospy.logerr(str(joints.get(frame).x)+" "+str(joints.get(frame).y)+" "+str(joints.get(frame).z))

                        t.transform.translation.x = joints.get(frame).x
                        t.transform.translation.y = joints.get(frame).y
                        t.transform.translation.z = joints.get(frame).z
                        t2.transform.translation.x = joints.get(frame).x * 2
                        t2.transform.translation.y = joints.get(frame).y * 2
                        t2.transform.translation.z = joints.get(frame).z * 2
                        t3.transform.translation.x = joints.get(frame).x * 3
                        t3.transform.translation.y = joints.get(frame).y * 3
                        t3.transform.translation.z = joints.get(frame).z * 3

                        # t.transform.translation.x = joints.get(frame).radius * math.sin(
                        #     joints.get(frame).pitch) * math.cos(joints.get(frame).yaw)
                        # t.transform.translation.y = joints.get(frame).radius * math.sin(
                        #     joints.get(frame).pitch) * math.sin(joints.get(frame).yaw)
                        # t.transform.translation.z = joints.get(frame).radius * math.cos(joints.get(frame).yaw)
                        # t2.transform.translation.x = joints.get(frame).radius * 2 * math.sin(
                        #     joints.get(frame).pitch) * math.cos(joints.get(frame).yaw)
                        # t2.transform.translation.y = joints.get(frame).radius * 2 * math.sin(
                        #     joints.get(frame).pitch) * math.sin(joints.get(frame).yaw)
                        # t2.transform.translation.z = joints.get(frame).radius * 2 * math.cos(joints.get(frame).yaw)

                        br = tf2_ros.TransformBroadcaster()
                        br2 = tf2_ros.TransformBroadcaster()
                        br3 = tf2_ros.TransformBroadcaster()

                        br.sendTransform(t)
                        br2.sendTransform(t2)
                        br3.sendTransform(t3)
                        dp.publish(dv)
                    else:
                        # rospy.logerr("None")
                        pass
        for d in dongle_list:
            tsa.TSDongle.close(d)
        print publishers
        print dv_publishers
        print broadcasters
        exit()


if __name__ == "__main__":
    SinglePublisher()
