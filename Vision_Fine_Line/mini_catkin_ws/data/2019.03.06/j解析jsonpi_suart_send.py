# -*- coding: utf-8 -*
import rospy
import serial
import time
import binascii  
import struct
import string
import json 
matrix=[5]
# 打开串口
ser = serial.Serial("/dev/ttyAMA0", 115200)
def main():
    while True:
        # 获得接收缓冲区字符
        time.sleep(0.8)
        count = ser.inWaiting()
        print count
        if count != 0:
            # 读取内容并回显
            recv = ser.read(count)
            ser.write(recv)
            print recv
            # print(recv["content"]["targetPos"])
            #json.dump(json_info,file)
            info = json.loads(recv)
            print(info["content"]["passPos"])
        # 清空接收缓冲区
        ser.flushInput()
        print ser.baudrate#波特率
        print ("He is %d years old",matrix[0])
        ser.write(str("a{}"))#向端口些数据
        # 必要的软件延时
        time.sleep(0.1)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()

