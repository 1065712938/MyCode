#!/usr/bin/env python
# Copyright 2017 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
import serial
import rospy
import string
import math
import time
import sys
ser = serial.Serial("/dev/ttyAMA0", 115200)
def main():
    while True:
         print('ww')
         time.sleep(0.1)
if __name__ == '__main__':
        main()

'''

# -*- coding: utf-8 -*
import rospy
import serial
import time
import binascii  
import struct
import string
import json  
from geometry_msgs.msg import Twist
matrix=[4]
hex_matrix = [1]
ser = serial.Serial("/dev/ttyAMA0", 115200)
def find_string(s,t):
    try:
        string.index(s,t)
        return True
    except(ValueError):
        return False 

def Analysis_json(recv):
     info = json.loads(recv)
     print(info)

def Exclude_FA(bytes):
    FA = 'fa'.decode('hex')
    for i in bytes:
      if i.encode('hex') == 'fa':
        print i
        bytes=bytes.translate(None,FA)
    Analysis_json(bytes) 

replce_str = ''
n = 'aaa'

def print_hex(bytes):
    FA = 'fa'.decode('hex')
    global replce_str
    
    global n
    print "KKKKKKKKKKKKKKKKKKKK",n
    n = n+'b'
    if find_string(bytes,FA) == False :
        print 'date inter'
        replce_str = ''
        replce_str = bytes
    else:
         print 'copy1', replce_str
         print 'OOOOOKKKK'
         replce_str =replce_str+bytes
         print 'copy', replce_str
         print 'nihao '
         replce_str1 = replce_str
         replce_str = ''
         Exclude_FA(replce_str1)
         replce_str = ''
         print 'over'
    for i in bytes:
     if i.encode('hex') == 'fa':
        print i
        bytes=bytes.translate(None,FA)
        print 'OOOOOOLLLLLLLLLL'
    l = [i.encode('hex') for i in bytes]
    print(" ".join(l))

def Deal_with_usart():
    count = ser.inWaiting()
    print count
    if count != 0:
          recv = ser.read(count)
          #ser.write(recv)

          print"recv22",recv
          print_hex(recv)
    ser.flushInput()
#    print ser.baudrate
    print ("send value is ")
   # ser.write((hex(int(Twist.linear.x))))
def PoseCallBack(Twist):
    count = ser.inWaiting()
    print count
    if count != 0:
          recv = ser.read(count)
          #ser.write(recv)

          print"recv22",recv
          print_hex(recv)
    ser.flushInput()
#    print ser.baudrate
    print ("send value is %x",(Twist.linear.x))
   # ser.write((hex(int(Twist.linear.x))))
    ser.write((str(int(Twist.linear.x))))

def main():
     rospy.init_node('state_sub',anonymous=False) 
     rospy.Subscriber('/state_val',Twist,PoseCallBack)
     #rospy.spin()
     while 1:
       Deal_with_usart()
       #rospy.spin()
       print 'OOOKKK'
       time.sleep(0.5)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if ser != None:

            ser.close()
