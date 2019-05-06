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
    num_flag = 0
    Xor_Num  = 0
    Xor_Num_Hex  = 0x00
    Checksum2 = 0
    for i in bytes1:
      num_flag = num_flag+1
      if num_flag<12:
         Xor_Num = Xor_Num^int(i.encode('hex'),16)
         #print("i ",i)
         print(" ",i.encode('hex'))
      else:
         print("Xor_Num1 ",Xor_Num)
         Checksum2 = 256-(Xor_Num+1)
         print("Checksum2 ",Checksum2)
         print(" ",i.encode('hex'))
         print(" Checksum",int(i.encode('hex'),16))
         if Checksum2 == int(i.encode('hex'),16):
		 All_Date = bytes1.encode('hex')
		 Date = All_Date[12:22]
                 print Date
		 Send_SD.publish(Date)
         break

    
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
