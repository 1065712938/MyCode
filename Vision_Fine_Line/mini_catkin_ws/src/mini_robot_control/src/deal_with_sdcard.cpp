#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include "mini_robot_control/vision_function.h"
#include "mini_robot_control/trajectory_plannin.h"
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/PointCloud.h>
#include "std_msgs/String.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string> 
using namespace cv;
using namespace std;
void Get_SDindex(const std_msgs::String& W_card_number);
void Get_SD_PoseCallback(const std_msgs::String& card_number) 
{ 
   cout<<"card_number "<<card_number<<endl;
   oFile_init<<"card_number  "<<card_number<<endl;
   Get_SDindex(card_number);
} 
int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "sdcard");  
  ros::NodeHandle nh; 
  ros::Subscriber Get_Pose = nh.subscribe("Send_SD_Value", 1000, Get_SD_PoseCallback);
  oFile_init.open("deal_with_sdcard.csv",ios::out|ios::trunc);
  ros::Rate loop_rate(100);
  while(ros::ok()){
     ros::spinOnce();
     ROS_INFO("subscriber_vision_data runnning!");
    loop_rate.sleep();
  }
  oFile_init.close();
  return 0;
}


void Get_SDindex(const std_msgs::String& W_card_number)
{
  vector<string> sdcard;
  vector<string>::iterator position;
  sdcard.push_back("00fa5fd8c3");//0
  sdcard.push_back("00fad1d9c3");//1
  sdcard.push_back("00aee9e3c3");//2
  sdcard.push_back("00fb89d8c3");//3
  cout<<"I heard: [%s] "<<W_card_number.data.c_str()<<endl;
  cout<<"W_card_number"<<W_card_number<<endl;
  std_msgs::String str1 = W_card_number;
  position = find(sdcard.begin(),sdcard.end(),W_card_number.data.c_str());
   if(position!=sdcard.end())
   {
        int index = distance(sdcard.begin(),position);
        cout<<"index "<<index<<endl;
   }
   else
   {
        cout<<"does not exist"<<endl;
   }
}