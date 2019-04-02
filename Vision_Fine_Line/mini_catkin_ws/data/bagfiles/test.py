# coding:utf-8
#!/usr/bin/python
 
# Extract images from a bag file.
 
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
if __name__ == '__main__':
       print "hello, world!"
       #  image_creator = ImageCreator()
