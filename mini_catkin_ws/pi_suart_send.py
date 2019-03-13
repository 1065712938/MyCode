# -*- coding: utf-8 -*
import rospy
import serial
import time
import binascii  
import struct
import string
import json 
from std_msgs.msg import Header
#from custom_msg_topic.custom_msg import Danger
from geometry_msgs.msg import PoseArray
matrix=[5]
# 打开串口
#ser = serial.Serial("/dev/ttyAMA0", 115200)
def main():
    while True:
        # 获得接收缓冲区字符
        time.sleep(0.8)
        #count = ser.inWaiting()
        count = 1
        print count
        if count != 0:
            # 读取内容并回显
            #recv = ser.read(count)
            #ser.write(recv)
            #info = json.loads(recv)
            #print recv
            # print(recv["content"]["targetPos"])
            #json.dump(json_info,file)
            info = json.loads('{"command":"set_target","content":{"passPos":[{"x":1.0,"y":0.0},{"x":2.0,"y":0.0},{"x":3.0,"y":3.0},{"x":3.0,"y":4.0},{"x":5.0,"y":5.0}],"targetPos":{"x":5.0,"y":5.0}}}')
            print(info["content"]["targetPos"])
        # 清空接收缓冲区
        #ser.flushInput()
        #print ser.baudrate#波特率
        print ("He is %d years old",matrix[0])
        # 必要的软件延时
        time.sleep(0.1)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()

