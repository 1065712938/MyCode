# coding:utf-8
#!/usr/bin/python
# Extract images from a bag file.
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import time
import signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from geometry_msgs.msg import Twist
rgb_path = 'home/mini_catkin_ws/data/bagfiles/image/'
flagpp=[4]
get_control_value = [4]
value1 = [4]
value2 = [4]
value3 = [4]
def PoseCallBack(Twist):
    print ("send value is %x",(Twist.linear.x))


class ImageCreator():
      def __init__(self):
        print 'hello, world!'
        self.bridge = CvBridge()
        with rosbag.Bag('2019-04-30-15-27-36.bag','r') as bag:  #要读取的bag文件；
            for topic,msg,t in bag.read_messages():
                print 'camera1',topic
                if topic == "/get_control_value":
                   get_control_value[0] = msg.linear.x
                   value1[0] = msg.linear.y
                   value2[0] = msg.linear.z
                   value3[0] = msg.angular.x
                   print ' value3[0]', value3[0]
                if topic == "/camera1/grayimage": #图像的topic；
                        #print 'have the topic'
                        try:
                            print 'camera'
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")#"mono8"bgr8
                        except CvBridgeError as e:
                            print e
                        print 'camera1',topic
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        flagpp[0] = flagpp[0]+1
                        image_name = str(flagpp[0])+'-'+str(round(value3[0],2))+'-'+str(round(value1[0],2))+'-'+str(round(value2[0],5))+ ".png" #图像命名：时间戳.png
                        cv_image = cv2.putText(cv_image, str(round(get_control_value[0],3)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.imshow('image',cv_image)
                        #cv2.imwrite(image_name,cv_image)  #保存；
                        cv2.waitKey(10)#35
                elif topic == "camera/depth_registered/image_raw": #图像的topic；
                        print 'NO this topic'
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"16UC1")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png" #图像命名：时间戳.png
                        cv2.imwrite(depth_path + image_name, cv_image)  #保存；
                time.sleep(0.1)
if __name__ == '__main__':
       signal.signal(signal.SIGINT, quit)                                
       signal.signal(signal.SIGTERM, quit)
       print "hello, world11!"
       rospy.init_node('state_sub1',anonymous=False) 
       #rospy.Subscriber('/get_control_value',Twist,PoseCallBack)
       image_creator = ImageCreator()









