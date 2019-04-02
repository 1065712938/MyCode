# coding:utf-8
#!/usr/bin/python
# Extract images from a bag file.
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
rgb_path = 'home/mini_catkin_ws/data/bagfiles/image/'
flagpp=[4]
class ImageCreator():
      def __init__(self):
        print 'hello, world!'
        self.bridge = CvBridge()
        with rosbag.Bag('2019-04-02-16-35-52.bag', 'r') as bag:  #要读取的bag文件；
            for topic,msg,t in bag.read_messages():
                print 'camera1',topic
                if topic == "/camera/image": #图像的topic；
                        try:
                            print 'camera'
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"mono8")#"mono8"bgr8
                        except CvBridgeError as e:
                            print e
                        print 'camera1',topic
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        flagpp[0] = flagpp[0]+1
                        image_name = str(flagpp[0])+ ".png" #图像命名：时间戳.png
                        cv2.imshow('image',cv_image)
                        cv2.imwrite(image_name,cv_image)  #保存；
                        cv2.waitKey(5)
                elif topic == "camera/depth_registered/image_raw": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"16UC1")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png" #图像命名：时间戳.png
                        cv2.imwrite(depth_path + image_name, cv_image)  #保存；
if __name__ == '__main__':
       print "hello, world11!"
       image_creator = ImageCreator()
