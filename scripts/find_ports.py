#!/usr/bin/env python
import glob
import serial

def findPorts():
    result=[]
    temp_list = glob.glob('/dev/ttyA[A-Za-z]*')
    for a_port in temp_list:
        try:
            s = serial.Serial(a_port)
            s.close()
            result.append(a_port)
        except serial.SerialException:
            pass
    return result
