#!/usr/bin/env python
# -*- coding: utf-8 -*
# Python 串口接收 JSON解析运用 将得到的点云数据发布
import rospy
import serial
import time
import binascii  
import struct
import string
import json  
import sys                                                                  
import signal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
matrix=[4]
hex_matrix = [1]
ser = serial.Serial("/dev/ttyAMA0", 115200)
def find_string(s,t):
    try:
        string.index(s,t)
        return True
    except(ValueError):
        return False 


def send_pose_point(PassPose,targetPos):
       print ',PassPose',len(PassPose)
       set_pose = rospy.Publisher('set_robot_pose', PointCloud, queue_size=10)
       my_awesome_pointcloud = PointCloud()
       print ',PassPose',len(PassPose)
       for i in range(0,len(PassPose)):
           print 'i = ',i
           my_awesome_pointcloud.points.append(Point32(PassPose[i]["x"], PassPose[i]["y"], 0.0))
       my_awesome_pointcloud.points.append(Point32(targetPos["x"], targetPos["y"], 0.0))
       x =56.0
       #my_awesome_pointcloud.points.append(Point32(x, 1.0, 0.0))
       # my_awesome_pointcloud.points.append(Point32(2.0, 2.0, 0.0))
       #my_awesome_pointcloud.points.append(Point32(3.0, 3.0, 0.0))
       set_pose.publish(my_awesome_pointcloud)
def Analysis_json(recv):
   try:
     info = json.loads(recv)
     print 'fianl_info',info
     # print('passPos = ',info["content"]["passPos"][1]["x"])
     print('passPos = ',len(info["content"]["passPos"]))
     print('targetPos = ',info["content"]["targetPos"]["x"])
     send_pose_point(info["content"]["passPos"],info["content"]["targetPos"])
   except:
     print 'fail KKKKKKKKKKKKKKKKKKKKKKKKKKKKK'

def Exclude_FA(bytes):
    FA = 'fa'.decode('hex')
    for i in bytes:
      if i.encode('hex') == 'fa':
        print i
        bytes=bytes.translate(None,FA)
    Analysis_json(bytes) 

replce_str = ''
n = 'aaa'

def print_hex1(bytes):
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


def print_hex(bytes):
    FA = 'fa'.decode('hex')
    global replce_str
    global n
    print "KKKKKKKKKKKKKKKKKKKK",n
    n = n+'b'
    if find_string(bytes,FA) == False :
        print 'date inter'
        # replce_str = ''
        replce_str =replce_str+ bytes
    else:
         #print 'replce_str', replce_str
         #print 'OOOOOKKKK'
         replce_str =replce_str+bytes
         #print 'copy', replce_str
         #print 'nihao '
         print replce_str.find(FA)
         replce_str1 = replce_str[0:replce_str.find(FA)]
         replce_str = ''
         Exclude_FA(replce_str1)
         replce_str = ''
         print 'over'
def Deal_with_usart():
    count = ser.inWaiting()
    print count
    if count != 0:
          recv = ser.read(count)
          #ser.write(recv)
          #print"recv22",recv
          print_hex(recv)
    ser.flushInput()
    print ("send value is ")

def PoseCallBack(Twist):
    #ser.write(recv)
    print ("send value is %x",(Twist.linear.x))
    ser.write((str(int(Twist.linear.x))))

def main():
     signal.signal(signal.SIGINT, quit)                                
     signal.signal(signal.SIGTERM, quit)
     rospy.init_node('state_sub',anonymous=False) 
     rospy.Subscriber('/state_val',Twist,PoseCallBack)
     set_pose = rospy.Publisher('set_robot_pose', PointCloud, queue_size=10)
     #rospy.spin()
     flag = 1
     while flag == 1:
       Deal_with_usart()
       my_awesome_pointcloud = PointCloud()
       x =5.0
       my_awesome_pointcloud.points.append(Point32(x, 1.0, 0.0))
       my_awesome_pointcloud.points.append(Point32(2.0, 2.0, 0.0))
       my_awesome_pointcloud.points.append(Point32(3.0, 3.0, 0.0))
       #set_pose.publish(my_awesome_pointcloud)
       # rospy.spin()
       # send_pose_point()
       print 'OOOKKK'
       time.sleep(0.5)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if ser != None:

            ser.close()
