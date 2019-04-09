#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "mini_robot_control/vision_function.h"
//#include "vision_function.h"

#include <vector>
#include <stdio.h>
using namespace cv;
using namespace std;
typedef Point_<double> Point2d;
Point2d get_midlinear_point;
vector<Mat> HSVChannels;
cv::Mat Split_S_img= Mat(120, 160, CV_8UC1);//Mat(120, 160, CV_8UC1)
int g_nThresholdValue_GARY = 70;
const int discard_value = 1024;
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

std::vector<int> save_cols_mid_date;
int re_mid_linear = 80;//r若检测到小于两个点 则设置为中点值 如果将其放置函数外面则为上一次的保存值
int get_mid_linear(vector<int> filter_colsr)
{
    if(filter_colsr.size()> 2){
         for(int m = 0;m<filter_colsr.size()-1;m++)
            {
                    int Dy_cols = filter_colsr.at(m+1)-filter_colsr.at(m);
                    if((Dy_cols>10)&&(Dy_cols<50))
                    {
                        //save_cols_mid_date.push_back(filter_colsr.at(m));
                        //save_cols_mid_date.push_back(filter_colsr.at(m+1));
                        re_mid_linear = (filter_colsr.at(m+1)+filter_colsr.at(m))/2;
                        break;
                    }
            }
    }  
    else if(filter_colsr.size()== 2){
            if(abs(filter_colsr.at(0)-filter_colsr.at(1))>10)
              re_mid_linear = (filter_colsr.at(0)+filter_colsr.at(1))/2;
            else re_mid_linear = discard_value;      
    }
    else{
         re_mid_linear = discard_value;
    }
    return re_mid_linear;

}
 cv::Mat img_rgb;
 std::vector<int> save_cols_date;
 vector<Point2d> Mid_Linear_Points;
 int pre_mid_linear = 80;
 void  Fine_Center_Line(cv::Mat frame)
 {
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
             // cout << "第" << i << "行" << j << "列灰度值：" << value << endl;
            }
        }
        //cout<<"i = "<<i<<"  save_cols_date.s = "<<save_cols_date.size()<<endl;
            mid_linear = get_mid_linear(save_cols_date);
            if(mid_linear!=discard_value)
            {
                if(flag_first == 0)
                  pre_mid_linear = mid_linear;//此处易存在bug
                flag_first = 1;
                //cout<<"i = "<<i<<"abs(m-p = "<<abs(mid_linear-pre_mid_linear)<<" mid_linear= "<<mid_linear<<" pre_mid_linear = "<<pre_mid_linear<<endl;
                if(abs(mid_linear-pre_mid_linear)>30)
                mid_linear = pre_mid_linear;
                pre_mid_linear = mid_linear;
                get_midlinear_point.x = mid_linear;
                get_midlinear_point.y = i;
                Mid_Linear_Points.push_back(get_midlinear_point);
            }
        //cv::circle(img_rgb,cv::Point2i(mid_linear,i),1,cv::Scalar(255,0,0),-1,2);
        save_cols_date.clear();
        i=i+5;
    }
    //cout<<"Mid_Linear_Points.size() = "<<Mid_Linear_Points.size()<<endl;
    for(int i = 0; i < Mid_Linear_Points.size(); i++)
     {

        cv::circle(img_rgb,cv::Point2i(Mid_Linear_Points.at(i).x,Mid_Linear_Points.at(i).y),2,cv::Scalar(255,0,0),-1,2);
        // oFile_init<<" x = "<<point3.at(i).x<<" y = "<<point3.at(i).y<<" z = "<<point3.at(i).z<<endl;
     }
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
