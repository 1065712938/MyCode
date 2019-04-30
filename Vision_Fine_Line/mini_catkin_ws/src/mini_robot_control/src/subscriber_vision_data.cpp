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
const float Robot_L= 0.18;//两个轮子之间的距离
int serial_number = -1;
float Arc_P = 0.04;
float Arc_I = 40;
float Arc_D = 0.0;
float Beeline_P = 0.01;
float Beeline_I = 100;
float Beeline_D = 0.0;
float Mini_speed = 0.15;
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
            if(Contral_Value<=0) Contral_Value = -Speed_Value*0.7;
            else  Contral_Value = Speed_Value*0.7;
         }

     }
     if(Contral_Value>0.1)  Contral_Value =  0.1;
     if(Contral_Value<-0.1) Contral_Value = -0.1;
     return Contral_Value;
}
float Model_Control(float Speed_Value,float Slope_Value)
{
     float R = 0;
     float adjust = 0;//0.0225 speed = 0.1
     if(abs(Slope_Value)<10)
       R = (2*Slope_Value)/(Slope_Value*Slope_Value+0.2304)*20;
     else R = (2*Slope_Value)/(Slope_Value*Slope_Value+0.2304)*2;
     adjust = -(Robot_L*Speed_Value)/(2*R);
     //if(Slope_Value>0) adjust = -adjust;
     cout<<"R = "<<R<<" adjust = "<<adjust<<endl;
     return adjust;
}
float Judge_Curve(float &Speed_Value,float Arc_K,float Linear_K)
{ 
     //0.8 
     if((abs(Arc_K)<0.7)&&(abs(Linear_K)<20))//(abs(Slope_Value)<10)||
     {
        cout<<"zhixian"<<"Arc_K = "<<Arc_K<<" Linear_K = "<<Linear_K<<endl;
        Speed_Value = Mini_speed*1.5;// *1.5
        pid.Kp= Beeline_P*Speed_Value*8;//0.16 度直线参数
        pid.Ki= Beeline_I;//0.002
        pid.Kd= Beeline_D;// 1*Speed_Value*5
        return 0;
     }
     else{
        cout<<"huxian"<<"Arc_K = "<<Arc_K<<" Linear_K = "<<Linear_K<<endl;
        Speed_Value = Mini_speed*1.3;// *1.3
        pid.Kp= Arc_P*Speed_Value*8;//0.16 曲线参数
        pid.Ki= Arc_I;//0.03
        pid.Kd= Arc_D;// 1*Speed_Value*5
        return 1;
     }
}
float Outside_Speed_Constant(float &Speed_Value,float Arc_Flag,float Linear_K,float adjust_amcl_Y)
{
    if((Arc_Flag == 1)&&(Speed_Value>0.25))
     {
        if(Linear_K>20)  
        {
          if(abs(adjust_amcl_Y*2)>Speed_Value)
          adjust_amcl_Y = -(Speed_Value*0.4);
          Speed_Value=Speed_Value+adjust_amcl_Y;

        } 
        if(Linear_K<-20)  
        {
           if(abs(adjust_amcl_Y*2)>Speed_Value)
           adjust_amcl_Y = (Speed_Value*0.4);
           Speed_Value=Speed_Value-adjust_amcl_Y;
        } 
      }
      return Speed_Value;
}
void Along_Linear_travel(vision_processing &VPSend_speed,float Slope_Value,float Arc_K,float Linear_K)
{
    //vision_processing *VP_sendSpeed=vision_processing::getInstance();
    //vision_processing VP_sendSpeed;
    float Speed_Value = Mini_speed;
    char Arc_Flag    = 0;
    Arc_Flag = Judge_Curve(Speed_Value,Arc_K,Linear_K);
    adjust_amcl_Y       = PID_Realize_Improve(0,Slope_Value);
     // adjust_amcl_Y       = Model_Control(Speed_Value,Slope_Value);
    adjust_amcl_Y = Limit_Max_Min(adjust_amcl_Y,Speed_Value,Slope_Value);
//adjust_amcl_Y = Limit_Max_Min(adjust_amcl_Y,Speed_Value,Slope_Value);
     VPSend_speed.Send_speed(Speed_Value,adjust_amcl_Y);
}

void get_init_param()
{
  ros::NodeHandle nh_private_("~");
  nh_private_.getParam("Arc_P", Arc_P);
  nh_private_.getParam("Arc_I", Arc_I);
  nh_private_.getParam("Arc_D", Arc_D);
  nh_private_.getParam("Beeline_P", Beeline_P);
  nh_private_.getParam("Beeline_I", Beeline_I);
  nh_private_.getParam("Beeline_D", Beeline_D);
  nh_private_.getParam("Mini_speed", Mini_speed);
  cout<<"Mini_speed = "<<Mini_speed<<"Arc_P = "<<Arc_P<<" Arc_I = "<<Arc_I<<" Arc_D = "<<Arc_D
  <<" Beeline_P = "<<Beeline_P<<" Beeline_I = "<<Beeline_I<<" Beeline_D = "<<Beeline_D<<endl;
}
int main(int argc, char **argv)  
{  
  cout<<"OO2222"<<endl;
  ros::init(argc, argv, "image_subscriber1");  
  ros::NodeHandle nh; // ("~")
  //ros::NodeHandle nh_private_("~"); 
  cout<<"OOKK"<<endl;
  get_init_param();
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
     double Linear_K = VP_Control.fit_lnear(fit_line_points);
     double Arc_K= VP_Control.Get_Deviation_Lnear(fit_line_points);
     double K = VP_Control.Get_Deviation_MidLnear(fit_line_points);
     cout<<" K = "<<K<<" Arc_K = "<<Arc_K<<endl;
     get_control_value.linear.x = K;
     get_control_value.linear.y = Arc_K;
     get_control_value.linear.z = adjust_amcl_Y;
     get_control_value.angular.x = Linear_K;
     send_get_control_value.publish(get_control_value);
    // if(abs(K)<2) K = 0;
    //  else{
    //    if(K>0)
    //    K = K - 2;
    //    else K = K + 2;
    //  }
     Along_Linear_travel(VP_Control,K,Arc_K,Linear_K);
     cout<<"K = "<<K<<endl;
     oFile_init<<"K "<<K<<" Arc_K "<<Arc_K<<" L_K "<<Linear_K<<"  adjust_amcl_Y "<<adjust_amcl_Y<<endl;
     ros::spinOnce();
     ROS_INFO("subscriber_vision_data runnning!");
    // ROS_INFO("Serial was %d",Arc_P);
    if(waitKey(20) >=0) 
      break;
    loop_rate.sleep();
  }
  oFile_init.close();
  ros::spin();  //等待message的到来
  //cv::destroyWindow("view");  
  return 0;
} 
