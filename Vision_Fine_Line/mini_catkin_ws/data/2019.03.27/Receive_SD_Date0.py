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
ser = serial.Serial("/dev/ttyAMA0",9600)
def PoseCallBack(Twist):
    count = ser.inWaiting()
    print count
    if count != 0:
          recv = ser.read(count)
          ser.write(recv)
          print recv
          info = json.loads(recv)
          print(info["content"]["passPos"])
    ser.flushInput()
#    print ser.baudrate
    print ("send value is %x",(Twist.linear.x))
   # ser.write((hex(int(Twist.linear.x))))
    ser.write((str(int(Twist.linear.x)))) 
def print_hex(bytes1):
    for i in bytes1:
      print(" ",i.encode('hex'))
    aa = bytes1.encode('hex')
    date = aa[10:22]
    print date
   
    #print('0x%x'bytes)
def main():
     #rospy.init_node('state_sub',anonymous=False) 
     #rospy.Subscriber('/state_val',Twist,PoseCallBack)
     while 1:
       count = ser.inWaiting()
       print count
       if count != 0:
          recv = ser.read(count)
         # ser.write(recv)
          print recv
          print_hex(recv)
      # Deal_with_usart()
      # my_awesome_pointcloud = PointCloud()
       # rospy.spin()
       print 'OOOKKK'
       ser.flushInput()
       time.sleep(0.6)

     rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if ser != None:

            ser.close()
