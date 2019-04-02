#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
  cv_bridge::CvImagePtr cv_ptr;  
  try  
  {  
     //mono8:  CV_8UC1， 灰度图像
     //把一个ROS 的sensor_msgs /image 信息转化为一cvimage
     //tocvcopy复制从ROS消息的图像数据，可以自由修改返回的cvimage
     cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");  //mono8
  }  
  catch (cv_bridge::Exception& e)  
  {  
     ROS_ERROR("cv_bridge exception: %s", e.what());  
     return;  
  }  

  cv::Mat img_rgb;  
  img_rgb = cv_ptr->image;  

  cv::imshow("view", img_rgb);  
  // fail if don't have waitKey(3).
  cv::waitKey(3);
}  

int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "image_subscriber");  
  ros::NodeHandle nh;  
  //然后打开一个图像显示窗口,并单独安排一个线程显示图像 
  cv::namedWindow("view", CV_WINDOW_NORMAL);   
  cv::startWindowThread();  
  image_transport::ImageTransport it(nh);  
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();  //等待message的到来
  cv::destroyWindow("view");  
  return 0;
} 
