#!/usr/bin/env python
import rospy
import serial
import time
import binascii  
import struct
import string
import json
import signal
from std_msgs.msg import String  
from geometry_msgs.msg import Twist
matrix=[4]
hex_matrix = [1]
ser = serial.Serial("/dev/ttyAMA0",9600)
def print_hex(bytes1):
    Send_SD = rospy.Publisher('Send_SD_Value', String, queue_size=10)
    for i in bytes1:
      print(" ",i.encode('hex'))
    All_Date = bytes1.encode('hex')
    Date = All_Date[12:22]
    Send_SD.publish(Date)
    print Date
def main():
     signal.signal(signal.SIGINT, quit)                                
     signal.signal(signal.SIGTERM, quit)
     rospy.init_node('state_sub',anonymous=False) 
     #rospy.Subscriber('/state_val',Twist,PoseCallBack)
     Send_SD = rospy.Publisher('Send_SD_Value', String, queue_size=10)
     while 1:
       count = ser.inWaiting()
       print count
       if count != 0:
          recv = ser.read(count)
          print_hex(recv)
       # rospy.spin()
       print 'OOOKKK'
       ser.flushInput()
       time.sleep(0.1)

     rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()
