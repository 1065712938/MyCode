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
float adjust_amcl_Y = 0;
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
      cout<<"OOK"<<endl;
     // cv::waitKey(3);
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

float Limit_Max_Min(float Contral_Value,float Speed_Value,float K)
{
       if(abs(Contral_Value)>Speed_Value/2.0){
         if(K<10)
         {
            if(Contral_Value<=0) Contral_Value = -Speed_Value/2.0;
            else  Contral_Value = Speed_Value/2.0;
         }
         else{
            if(Contral_Value<=0) Contral_Value = -Speed_Value;
            else  Contral_Value = Speed_Value;
         }

     }
     return Contral_Value;
}
void Along_Linear_travel(vision_processing &VPSend_speed,float Slope_Value)
{
    //vision_processing *VP_sendSpeed=vision_processing::getInstance();
    //vision_processing VP_sendSpeed;
     float Speed_Value = 0.25;
     if(Slope_Value<10)
     {
        pid.Kp= 0.01*Speed_Value*8;//0.16 度直线参数
        pid.Ki=0.002;//
        pid.Kd=0;// 1*Speed_Value*5
     }
     else{
        pid.Kp= 0.05*Speed_Value*8;//0.16 度直线参数
        pid.Ki=0.03;//
        pid.Kd=0;// 1*Speed_Value*5
     }
      adjust_amcl_Y       = PID_Realize_Improve(0,Slope_Value);
     
     cout<<"Slope_Value = "<<Slope_Value<<"  adjust_amcl_Y = "<<adjust_amcl_Y<<endl;
     adjust_amcl_Y = Limit_Max_Min(adjust_amcl_Y,Speed_Value,Slope_Value);
     VPSend_speed.Send_speed(Speed_Value,adjust_amcl_Y);
}
int main(int argc, char **argv)  
{  
  cout<<"OO2222"<<endl;
  ros::init(argc, argv, "image_subscriber1");  
  ros::NodeHandle nh;  
  cout<<"OOKK"<<endl;
  //cv::namedWindow("view", CV_WINDOW_NORMAL);  //然后打开一个图像显示窗口,并单独安排一个线程显示图像  
  //cv::startWindowThread();  
  image_transport::ImageTransport it(nh);  
  //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::Subscriber Get_Pose = nh.subscribe("Linear_Point", 1000, Get_Linear_PoseCallback);
  ros::Publisher send_get_control_value = nh.advertise<geometry_msgs::Twist>("get_control_value",2);//改为1
  geometry_msgs::Twist get_control_value;
  cout<<"OOKK11"<<endl;
  vision_processing VP_Control;
  cout<<"OOKK1122"<<endl;
  oFile_init.open("subscriber_vision_data.csv",ios::out|ios::trunc);
  PID_init();
  ros::Rate loop_rate(100);
  cout<<"OOKK1133"<<endl;
  while(ros::ok()){
     double K1 = VP_Control.fit_lnear(fit_line_points);
     double Arc_K= VP_Control.Get_Deviation_Lnear(fit_line_points);
     double K = VP_Control.Get_Deviation_MidLnear(fit_line_points);
     cout<<" K = "<<K<<" Arc_K = "<<Arc_K<<endl;
     get_control_value.linear.x = K;
     get_control_value.linear.y = Arc_K;
     get_control_value.linear.z = adjust_amcl_Y;
     send_get_control_value.publish(get_control_value);
     if(abs(K)<2) K = 0;
     else{
       if(K>0)
       K = K - 2;
       else K = K + 2;
     }
     Along_Linear_travel(VP_Control,K);
     cout<<"K = "<<K<<endl;
     oFile_init<<"K "<<K<<" Arc_K "<<Arc_K<<" adjust_amcl_Y "<<adjust_amcl_Y<<endl;
     ros::spinOnce();
     ROS_INFO("subscriber_vision_data runnning!");
    if(waitKey(20) >=0) 
      break;
    //loop_rate.sleep();
  }
  oFile_init.close();
  ros::spin();  //等待message的到来
  //cv::destroyWindow("view");  
  return 0;
} 
