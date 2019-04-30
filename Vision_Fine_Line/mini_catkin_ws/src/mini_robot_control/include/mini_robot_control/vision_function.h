#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>

//#include <opencv2/legacy/compat.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
extern int g_nThresholdValue_GARY;
extern const int Frame_Width;
extern const int Frame_Height;
class vision_processing
{

//private:
  // static vision_processing *instance;
  // vision_processing()
  // {
  //   Send_Linear_Point = linear_nh.advertise<sensor_msgs::PointCloud>("/Linear_Point", 10);
  //   Send_Request_Speed = linear_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); //改为1
  //   image_transport::ImageTransport it1(linear_nh1);
  //   //Publisher_Image = linear_nh.advertise<sensor_msgs::ImagePtr>("camera/image1", 1);
  //   Publisher_Image = it1.advertise("camera1/image1", 1);
  // }

public:
  // static vision_processing *getInstance()
  // {
  //   if (instance == nullptr)
  //   {
  //     instance = new vision_processing();
  //   }
  //   return instance;
  // }

  vision_processing()
  {
    Send_Linear_Point = linear_nh.advertise<sensor_msgs::PointCloud>("/Linear_Point", 10);
    Send_Request_Speed = linear_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); //改为1
    image_transport::ImageTransport it1(linear_nh);
    Publisher_Image = it1.advertise("camera1/grayimage", 1);
    image_transport::ImageTransport it2(linear_nh);
    Publisher_Fit_Image = it2.advertise("camera2/fitimage", 1);
  }
  sensor_msgs::ImagePtr Publisher_msg;
  void Read_Picture_W_Circle();
  void Deal_gray_Vision(cv::Mat frame);
  void Deal_HSV_Vision(cv::Mat frame);
  void print_amcl_linear_x_ahead1();
  void Send_stop();
  void fit_linear_fun_experiment();
  double fit_lnear(std::vector<cv::Point> Fit_Points);//
  double Get_Deviation_Lnear(std::vector<cv::Point> Fit_Points);
  double Get_Deviation_MidLnear(std::vector<cv::Point> Fit_Points);
  void Send_speed(float speed, float adjust);

private:
  ros::NodeHandle linear_nh;
  //ros::NodeHandle linear_nh1;
  ros::Publisher Send_Linear_Point;
  ros::Publisher Send_Request_Speed;
  image_transport::Publisher Publisher_Image;
  image_transport::Publisher Publisher_Fit_Image;

  ros::Subscriber VPsub_;
  void Fine_Center_Line(cv::Mat frame);
  void Draw_Line(std::vector<cv::Point> Fit_Points,cv::Vec4f _line_para);

};
void Read_Picture_W_Circle(); //cv::Mat img