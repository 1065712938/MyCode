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
class vision_processing
{
  public:
    vision_processing()
    {
        
        Send_Linear_Point = linear_nh.advertise<sensor_msgs::PointCloud>("/Linear_Point", 10);
        Send_Request_Speed = linear_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
     // ros::NodeHandle linear_nh1;
     // ros::Publisher send_Linear_Point = linear_nh1.advertise<sensor_msgs::PointCloud>("Linear_Point_vision_processing",1);
    }

    void Read_Picture_W_Circle();
    void Deal_gray_Vision(cv::Mat frame);
    void Deal_HSV_Vision(cv::Mat frame);
    void print_amcl_linear_x_ahead1();
    void Send_stop();
    void fit_linear_fun_experiment();
    double fit_lnear(std::vector<cv::Point> Fit_Points);
    void Send_speed(float speed ,float adjust);

 private:
        ros::NodeHandle linear_nh; 
        ros::Publisher  Send_Linear_Point;
        ros::Publisher  Send_Request_Speed;
        ros::Subscriber VPsub_;
        void Fine_Center_Line(cv::Mat frame);
};
void Read_Picture_W_Circle();//cv::Mat img