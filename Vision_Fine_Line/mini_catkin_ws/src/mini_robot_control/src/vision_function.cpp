#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "mini_robot_control/vision_function.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>

//#include "vision_function.h"

#include <vector>
#include <stdio.h>
using namespace cv;
using namespace std;
typedef Point_<double> Point2d;
Point2d get_midlinear_point;
std::vector<int> save_cols_date;
vector<Point2d> Mid_Linear_Points;
vector<Mat> HSVChannels;
cv::Mat img_rgb;
cv::Mat Split_S_img= Mat(120, 160, CV_8UC1);//Mat(120, 160, CV_8UC1)
int g_nThresholdValue_GARY = 70;
int pre_mid_linear = 80;
const int discard_value = 1024;
const int Filter_Point_Error_Value = 20;
const int Linear_Min = 10;
const int Linear_Max = 50;
void  Send_stop();


void  Send_stop()
{
    cout<<"Send_stop"<<endl;
    ros::NodeHandle robot_stop;
    ros::Publisher send_stop = robot_stop.advertise<geometry_msgs::Twist>("cmd_vel_stop",1);//改为1
    for(int i = 0;i<5;i++)
     {
        geometry_msgs::Twist twist;
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;//angle_adjust_linear
        send_stop.publish(twist);

     }
}


/*
函数名   : Read_Picture_W_Circle() 
功能     :  读取一张照片 并在照片中心 画圆
调用方式  :  
说明     : 照片必须位于运行代码行的目录下 不是代码文件所在的目录
*/
//原文：https://blog.csdn.net/Lee_Dk/article/details/80506559 
void vision_processing::Read_Picture_W_Circle()   //传值 
{
    cv::Mat image = cv::imread("airplane.jpg",CV_LOAD_IMAGE_COLOR);
    if(image.empty())
    {
        printf("open error\n");
    }
    circle(image,cv::Point2i(image.cols/2,image.rows/2),30,cv::Scalar(0,0,255),-1,8);
    imshow("img",image);
	cv::waitKey(3);
}


/*
函数名称 :  get_mid_linear 
调用方式 :  mid_linear = get_mid_linear(save_cols_date);
功能  :  得到每行的轮廓的中心点
说明  :  默认是从左向右寻找 两个点在[Linear_Min Linear_Max]之间的点
        也可以根据需要寻找从右到左的点（当优先考虑右侧的黑线时）
*/
std::vector<int> save_cols_mid_date;
int re_mid_linear = 80;//r若检测到小于两个点 则设置为中点值 如果将其放置函数外面则为上一次的保存值
int get_mid_linear(vector<int> filter_colsr)
{
    if(filter_colsr.size()> 2){
        //优先考虑从左到右的 满足的点
         for(int m = 0;m<filter_colsr.size()-1;m++)
            {
                    int Dy_cols = filter_colsr.at(m+1)-filter_colsr.at(m);
                    if((Dy_cols>Linear_Min)&&(Dy_cols<Linear_Max))
                    {
                        re_mid_linear = (filter_colsr.at(m+1)+filter_colsr.at(m))/2;
                        break;
                    }
            }
    }  
    else if(filter_colsr.size()== 2){
            int Dy_cols2 = abs(filter_colsr.at(0)-filter_colsr.at(1));
            if((Dy_cols2>Linear_Min)&&(Dy_cols2<Linear_Max))
              re_mid_linear = (filter_colsr.at(0)+filter_colsr.at(1))/2;
            else re_mid_linear = discard_value;      
    }
    else{
         re_mid_linear = discard_value;
    }
    return re_mid_linear;

}


/*
函数名     : print_vector2d 
功能       :  打印二维容器
调用方式    :  
说明       :
*/
void print_vector2d(vector<Point2d> point2)
{
     for(int i = 0; i < point2.size(); i++)
     {
         cout<<" x = "<<point2.at(i).x<<" y = "<<point2.at(i).y<<endl;
         //oFile_init<<" x = "<<point3.at(i).x<<" y = "<<point3.at(i).y<<" z = "<<point3.at(i).z<<endl;
     }
}

/*
函数名称 :  filter_Mid_Linear_Points 
调用方式 :  filter_Mid_Linear_Points(Mid_Linear_Points);
功能  :  过滤得到的线中心坐标
说明  :  改变原容器的值  效果：原73 77 44 48 70过滤后为 73 77 70

*/
void filter_Mid_Linear_Points(vector<Point2d> &_Mid_Linear_Points)
{
    vector<Point2d> Re_Mid_Linear_Points;
    //print_vector2d(_Mid_Linear_Points);
    for(int i = 1; i < (_Mid_Linear_Points.size()-1);)
     {
          float pre_z  = _Mid_Linear_Points.at(i-1).x;
          float cur_z  = _Mid_Linear_Points.at(i).x;
          float fut_z  = _Mid_Linear_Points.at(i+1).x;
          //cout<<"abs(c - p) = "<<abs(cur_z - pre_z)<<"abs(c - f) = "<<abs(cur_z - fut_z)<<endl;
          if((abs(cur_z - pre_z)<Filter_Point_Error_Value)&&(abs(cur_z - fut_z)<Filter_Point_Error_Value))
          {
            Re_Mid_Linear_Points.push_back(_Mid_Linear_Points.at(i));
            i++;
          }  
          else{
             // cout<<"vaule = "<<_Mid_Linear_Points.at(i).x<<endl;
              _Mid_Linear_Points.erase(_Mid_Linear_Points.begin()+i);
              //cout<<"vaule1 = "<<_Mid_Linear_Points.at(i).x<<endl;
          }
     }
    //print_vector2d(_Mid_Linear_Points);
   _Mid_Linear_Points.assign(Re_Mid_Linear_Points.begin(), Re_Mid_Linear_Points.end());
}


/*
函数名称 :  Fine_Center_Line 
调用方式 :  Fine_Center_Line(frame);
功能  :    巡线程序总入口
说明  :  

*/

 void  Fine_Center_Line(cv::Mat frame)
 {

    ros::NodeHandle linear_nh;
    ros::Publisher send_Linear_Point = linear_nh.advertise<sensor_msgs::PointCloud>("Linear_Point",1);//改为1
    cout<<"OK"<<endl;
    Canny(frame, frame, 3, 9, 3);
    cv::cvtColor(frame, img_rgb, CV_GRAY2RGB);
    int rows = frame.rows;
    int cols = frame.cols;
    int mid_linear = frame.cols/2;
    int flag_first = 0;
    for (int i = 0; i < rows;)
    {  
        uchar* ptr = (uchar*)frame.data +i *cols;
        for (int j = 0; j < cols; j++)
        {
            int value = ptr[j];
            if(value>0)
            {
              save_cols_date.push_back(j);
            }
        }
        mid_linear = get_mid_linear(save_cols_date);
        if(mid_linear!=discard_value)
        {
            pre_mid_linear = mid_linear;
            get_midlinear_point.x = mid_linear;
            get_midlinear_point.y = i;
            Mid_Linear_Points.push_back(get_midlinear_point);
        }
        save_cols_date.clear();
        i=i+5;
        ros::spinOnce();  
    }
    //cout<<"Mid_Linear_Points.size() = "<<Mid_Linear_Points.size()<<endl;
    if(Mid_Linear_Points.size()>2)
      filter_Mid_Linear_Points(Mid_Linear_Points);
    for(int i = 0; i < Mid_Linear_Points.size(); i++)
     {
        
        cv::circle(img_rgb,cv::Point2i(Mid_Linear_Points.at(i).x,Mid_Linear_Points.at(i).y),2,cv::Scalar(255,0,0),-1,2);
        //cout<<" x1 = "<<Mid_Linear_Points.at(i).x<<" y = "<<Mid_Linear_Points.at(i).y<<endl;
     }
   
    unsigned int num_points = Mid_Linear_Points.size();// 
    sensor_msgs::PointCloud _Mid_Linear_Points;
    _Mid_Linear_Points.points.resize(num_points);
    for(unsigned int i = 0; i < num_points; ++i){
        _Mid_Linear_Points.points[i].x = Mid_Linear_Points.at(i).x;
        _Mid_Linear_Points.points[i].y = Mid_Linear_Points.at(i).y;
        _Mid_Linear_Points.points[i].z = 0;
    }
    send_Linear_Point.publish(_Mid_Linear_Points);
    //先publish 再clear
    Mid_Linear_Points.clear();
    //cv::cvtColor(frame, img_rgb, CV_GRAY2RGB);
    //cv::circle(img_rgb,cv::Point2i(frame.cols/2,frame.rows/2),3,cv::Scalar(255,0,0),-1,2);
    cv::line(img_rgb,cv::Point2i(frame.cols/2,0),cv::Point2i(frame.cols/2,frame.rows), cv::Scalar(0,0,100), 1,CV_AA ); 
    imshow("frame_gray_Canny",img_rgb);
 }


/*
函数名     Deal_gray_Vision(cv::Mat frame)
功能       ：对图像的灰度值进行处理 提取白底黑线
调用方式    ：VPF.Deal_gray_Vision(frame);
说明       ：不改变原图像
*/
void vision_processing::Deal_gray_Vision(cv::Mat frame)
{ 
    cvtColor(frame, frame, CV_BGR2GRAY);
    //cv::imshow("frame_gray",frame) ;
    threshold(frame, frame, g_nThresholdValue_GARY, 255, THRESH_BINARY);
    cv::medianBlur(frame,frame,3);
    imshow("frame_gray_bool",frame);
    Fine_Center_Line(frame);
}

/*
函数名      ：Deal_gray_Vision(cv::Mat frame)
功能       ：对图像的HSV 提取S进行处理 提取白底黑线
调用方式    ：VPF.Deal_HSV_Vision(frame);
说明       ：不改变原图像
*/
void vision_processing::Deal_HSV_Vision(cv::Mat frame)
{
    cvtColor(frame, frame, CV_BGR2HSV);
    split(frame, HSVChannels); //分离通道
    resize(HSVChannels.at(1), Split_S_img, Split_S_img.size());
    cv::imshow("frame_S",Split_S_img);
    threshold(Split_S_img, Split_S_img, g_nThresholdValue_GARY, 255, THRESH_BINARY_INV);
    imshow("frame_S_bool",Split_S_img);
}

void vision_processing::print_amcl_linear_x_ahead1()
{
  cout<<"Hello SLAM amcl_linear_x_ahead"<<endl;
}
