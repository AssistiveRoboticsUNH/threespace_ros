#!/usr/bin/env python
import roslib
import rospy
import serial
import matlab.engine
import time
import math
import collections
import StringIO as io
import numpy as np
from gait_hmm_ros.msg import imu_vector
from gait_hmm_ros.msg import ardu_msg
from threespace_ros.msg import dataVec
from  threespace_ros.msg import GenericFloatArray


def imuCallback(msg, args):
    self = args[0]
    self.refresh = True
    index = args[1]
    topic = args[2]
    startIndex = index * ((3 * self.useAccel) + (3 * self.useGyro))
    currentIndex = 0
    # print("---------------------------------")
    # print args[1]
    # print args[2]
    # print startIndex
    # rospy.logwarn('Topic Received : ', str(topic))
    # rospy.logwarn('Topic Index: ', str(index))
    # print args
    if self.useGyro == 1:
        self.imuVec[startIndex + currentIndex] = msg.gyroX
        currentIndex += 1
        self.imuVec[startIndex + currentIndex] = msg.gyroY
        currentIndex += 1
        self.imuVec[startIndex + currentIndex] = msg.gyroZ
        currentIndex += 1
    if self.useAccel == 1:
        self.imuVec[startIndex + currentIndex] = msg.accX
        currentIndex += 1
        self.imuVec[startIndex + currentIndex] = msg.accY
        currentIndex += 1
        self.imuVec[startIndex + currentIndex] = msg.accZ
    # print currentIndex + startIndex
    # print("---------------------------------")


def arduCallback(msg, args):
    self = args[0]
    if self.ir == 1:
        self.arVec[0] = msg.ir
    if self.prox == 1:
        self.arVec[0 + self.ir] = msg.prox
    if self.fsr == 1:
        self.arVec[self.ir + self.prox] = msg.fsrfl
        self.arVec[self.ir + self.prox + 1] = msg.fsrfr
        self.arVec[self.ir + self.prox + 2] = msg.fsrbk
    pass


class LiveBroadcaster:

    def createArray(self):
        pass

    def __init__(self, acb=arduCallback, imc=imuCallback):
        rospy.init_node('liveBroadCaster')
        self.acb = acb
        self.imc = imc

        self.combinedVector = []
        self.imuVec = []
        self.arVec = []

        self.matlabMsg = GenericFloatArray()

        self.useGyro = rospy.get_param("~use_gyro_n", 0)
        self.useAccel = rospy.get_param("~use_accel_n", 0)
        self.fsr = rospy.get_param("~fsr_n", 1)
        self.ir = rospy.get_param("ir_n", 1)
        self.prox = rospy.get_param("~prox_n", 1)
        self.imuCount = 0
        self.rfIndex = 0
        self.rllIndex = 0
        self.rulIndex = 0
        self.mIndex = 0
        self.imuIndexes = [-1, -1, -1, -1]

        out = io.StringIO()
        err = io.StringIO()

        self.window = rospy.get_param("~window_n", 0)
        self.thres = rospy.get_param("~thres_n", 0)

        self.rf = rospy.get_param("~rf_n", "")
        if self.rf != "":
            self.rfIndex = self.imuCount
            rfSub = rospy.Subscriber(self.rf, dataVec, self.imc, (self, self.rfIndex, self.rf))
            self.imuCount += 1

        self.rll = rospy.get_param("~rll_n", "")
        if self.rll != "":
            self.rllIndex = self.imuCount
            rllSub = rospy.Subscriber(self.rll, dataVec, self.imc, callback_args=(self, self.rllIndex, self.rll))
            self.imuCount += 1

        self.rul = rospy.get_param("~rul_n", "")
        if self.rul != "":
            self.rulIndex = self.imuCount
            rulSub = rospy.Subscriber(self.rul, dataVec, self.imc, callback_args=(self, self.rulIndex, self.rul))
            self.imuCount += 1

        self.m = rospy.get_param("~m_n", "")
        if self.m != "":
            self.mIndex = self.imuCount
            mSub = rospy.Subscriber(self.m, dataVec, self.imc, callback_args=(self, self.mIndex, self.m))
            self.imuCount += 1

        arduinoSub = rospy.Subscriber('arduino', ardu_msg, self.acb, callback_args=self)

        size = self.imuCount*(3*self.useAccel + 3*self.useGyro) + self.fsr + self.ir + self.prox
        self.imuVec = [-1337.1337 for i in range(0, size)]
        self.minVec = [0.0 for i in range(0, size)]
        self.maxVec = [0.0 for i in range(0, size)]
        self.avgVec = []

        matlabInput = collections.deque(maxlen=self.window)
        minMaxFound = False

        eng = matlab.engine.start_matlab()
        eng.load('~/full_data/trained_wss/subject3/GyroAccel/normal/subject3_gyro_accel'
                 '_fsr_prox_btr_bte_rf_rul_m_full_data_normalized_workspace.mat', nargout=0)
        eng.cd('~/ros_ws/src/threespace_ros/scripts')

        testVec = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0]

        # x = float(eng.evalfis(testVec, eng.an1, nargout=1, stdout=out, stderr=err))
        # eng.edit('triarea', nargout=0)

        x = eng.live_classifier(testVec, nargout=1)

        startTime = rospy.Time.now()
        r = rospy.Rate(100)
        d = rospy.Duration(0.2, 0)
        self.refresh = False
        trainDuration = 5
        while not rospy.is_shutdown():
            # rospy.spin()
            loopTime = rospy.Time.now()

            if(loopTime.to_sec() - startTime.to_sec()) > trainDuration:
                # make sure we have data from every sensor
                if -1337.1337 not in self.imuVec:
                    # have we found the limits for every input so far ?
                    if minMaxFound:
                        # if we have enough entries to classify (This should not matter
                        # a window of 1 should be fine, but we may want to average
                        # more than one outputs to avoid false classifications)
                        if len(matlabInput) < self.window:
                            # normalize the input data and pass it to matlab vector
                            self.imuVec = (self.imuVec - self.minVec)/(self.maxVec - self.minVec)
                            matlabInput.append(self.imuVec)
                        else:
                            # matlabInput is a deque so it will automatically
                            # pop the oldest elements to insert incoming vectors
                            print matlabInput
                        continue
                    # if not find min and max values form every input
                    # so we can calculate the average (very sloppy tbh)
                    else:
                        self.maxVec = np.amax(self.avgVec, axis=0)
                        self.minVec = np.amin(self.avgVec, axis=0)
                        self.avgVec.clear()
                        minMaxFound = True
                print len(self.avgVec)
                print len(self.avgVec[0])
                for i in range(len(self.avgVec) - trainDuration, len(self.avgVec)):
                    print self.avgVec[i][0:6], i
                    self.maxVec = np.amax(self.avgVec, axis=0)
                    self.minVec = np.amin(self.avgVec, axis=0)
                print self.maxVec
                print self.minVec
                eng.quit()
                rospy.signal_shutdown()
                exit()
            else:
                if self.refresh:
                    print self.imuVec[0:6]
                    self.avgVec.append(self.imuVec[:])
                    self.refresh = False
            # r.sleep()
            rospy.sleep(d)
        eng.quit()

if __name__ == '__main__':
    try:
        LiveBroadcaster()
    except rospy.ROSInterruptException:
        pass
