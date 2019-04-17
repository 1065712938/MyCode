#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include "mini_robot_control/vision_function.h"
#include "mini_robot_control/trajectory_plannin.h"
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/PointCloud.h>
#include <fstream>
#include <sstream>
#include <iostream>
using namespace cv;
using namespace std;

std::vector<cv::Point> Creat_vector_point;
std::ofstream oFile_init;
void Creat_vector_pointfun()
{
  for(int i = 0;i<120;)
  {
   	Creat_vector_point.push_back(cv::Point(i, i));
    i = i+5;
  }
  // for(int i = 120;i>0;)
  // {
  //   Creat_vector_point.push_back(cv::Point(i, -i));
  //   i = i-5;
  //  }

}

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
     // cv::imshow("view", img_rgb);  
      // fail if don't have waitKey(3).
      cv::waitKey(3);
}  

sensor_msgs::PointCloud g_Set_Points;
vector<Point> fit_line_points;
void Get_Linear_PoseCallback(const sensor_msgs::PointCloud& Points) 
{
    fit_line_points.clear();
    for(unsigned int i = 0; i < Points.points.size(); i++)
    {
      fit_line_points.push_back(cv::Point(Points.points[i].x, Points.points[i].y));
    }
}

float Limit_Max_Min(float Contral_Value,float Speed_Value)
{
       if(abs(Contral_Value)>Speed_Value/2.0){
            if(Contral_Value<=0) Contral_Value = -Speed_Value/2.0;
            else  Contral_Value = Speed_Value/2.0;
     }
     return Contral_Value;
}
void Along_Linear_travel(vision_processing &VPSend_speed,float Slope_Value)
{
    //vision_processing *VP_sendSpeed=vision_processing::getInstance();
    //vision_processing VP_sendSpeed;

     pid.Kp=0.05;//0.16 度直线参数
     pid.Ki=0.03;
     pid.Kd=0.2;
     float adjust_amcl_Y       = PID_realize(1,Slope_Value);
     float Speed_Value = 0.1;
     cout<<"adjust_amcl_Y = "<<adjust_amcl_Y<<endl;
     adjust_amcl_Y = Limit_Max_Min(adjust_amcl_Y,Speed_Value);
    // VP_sendSpeed.Send_speed(Speed_Value,adjust_amcl_Y);
     VPSend_speed.Send_speed(Speed_Value,adjust_amcl_Y);
}
int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "image_subscriber");  
  ros::NodeHandle nh;  
  cv::namedWindow("view", CV_WINDOW_NORMAL);  //然后打开一个图像显示窗口,并单独安排一个线程显示图像  
  cv::startWindowThread();  
  image_transport::ImageTransport it(nh);  
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::Subscriber Get_Pose = nh.subscribe("Linear_Point", 1000, Get_Linear_PoseCallback);
  vision_processing VP_Control;
  oFile_init.open("subscriber_vision_data.csv",ios::out|ios::trunc);
  PID_init();
  while(ros::ok()){
     //double K = VP_Control.fit_lnear(fit_line_points);
     double K = VP_Control.Get_Deviation_Lnear(fit_line_points);
     //Along_Linear_travel(VP_Control,K);
     cout<<"K = "<<K<<endl;
     oFile_init<<"K = "<<K<<endl;
     ros::spinOnce();
     ROS_INFO("subscriber_vision_data runnning!");
    if(waitKey(20) >=0) 
      break;
  }
  oFile_init.close();
  ros::spin();  //等待message的到来
  cv::destroyWindow("view");  
  return 0;
} 
