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
typedef Point_<int> Point2dint;
Point2d get_midlinear_point;
std::vector<int> save_cols_date;
std::vector<int> save_few_cols_date;
vector<Point2d> Mid_Linear_Points;
vector<vector<Point2d> > Mid_Linear_Points_2d;

vector<Mat> HSVChannels;
#define No_Cross_TurnMid    0
#define Turn_Left           1
#define Turn_Right          2

cv::Mat img_rgb;
cv::Mat Split_S_img= Mat(120, 160, CV_8UC1);//Mat(120, 160, CV_8UC1)
const int Frame_Width = 160;//140
const int Frame_Height = 140;//140
int Select_Traversal_Mode = Turn_Right;
int Select_Linear = Turn_Right;//向左转时 确保已转过后 再将其设置为0
int g_nThresholdValue_GARY = 70;
int pre_mid_linear = Frame_Width/2.0;
const int discard_value = 1024;
const int Filter_Point_Error_Value = 40;//20
const int Linear_Min = 10;
const int Linear_Max = 60;//60
const int Filter_X   = 0;
const int Filter_Y   = 1;
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
函数名     : print_vector 
功能       :  打印一维容器
调用方式    :  
说明       :
*/
void print_vector(vector<int> point2)
{
     for(int i = 0; i < point2.size(); i++)
     {
         cout<<" x = "<<point2.at(i)<<endl;
         //oFile_init<<" x = "<<point3.at(i).x<<" y = "<<point3.at(i).y<<" z = "<<point3.at(i).z<<endl;
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
函数名    :Select_Linear_fun 
功能      :  根据SD选择需要跟随的黑线
调用方式  :  
说明     :使用数组保存每条线的中心坐标
*/
double rep_mid_linear_value = Frame_Width/2.0;
double Select_Linear_fun(vector<int> filter_colsr)
{
    int linears = 0;
    double save_linear_array[10];
    for(int m = 0;m<filter_colsr.size()-1;)
    {
            int Dy_cols = abs(filter_colsr.at(m+1)-filter_colsr.at(m));
            if((Dy_cols>Linear_Min)&&(Dy_cols<Linear_Max))
            {
                save_linear_array[linears] = (filter_colsr.at(m+1)+filter_colsr.at(m))/2;
                linears++;
                m += 2;
                rep_mid_linear_value = save_linear_array[0];
            }
            else m++;
    }
    //cout<<"linears = "<<linears<<endl;
    //print_vector(filter_colsr);
    // if((linears - 1)<Select_Linear) 
    // {
    //   return discard_value;
    // }
    if(Select_Linear == 0)
    {
        if(linears == 0) return rep_mid_linear_value;
        else if(linears == 1)  return save_linear_array[0];
        else 
        {
            //cout<<"Select_LinearSelect_LinearSelect_Linear"<<endl;
            return save_linear_array[0];
        }
    }
    else
    {
        if(linears == 0) return discard_value;
        return save_linear_array[0];
    }

}



/*
函数名称 :  filter_Mid_Linear_Value 
调用方式 :  filter_Mid_Linear_Points(Mid_Linear_Points);
功能  :  
说明  :  改变原容器的值  效果：43 44 70 71 得到44 71
*/
void filter_Mid_Linear_Value(vector<int> &_Mid_Linear_Points)
{
    vector<int> Re_Mid_Linear_Points;
    // print_vector(_Mid_Linear_Points);
     int continu_value = 5;
    for(int i = 0; i < (_Mid_Linear_Points.size()-1); i++)
     {
          float cur_z  = _Mid_Linear_Points.at(i);
          float fut_z  = _Mid_Linear_Points.at(i+1);
          if(abs(cur_z - fut_z)>continu_value)
          {
            Re_Mid_Linear_Points.push_back(_Mid_Linear_Points.at(i));
          }  
      }
     Re_Mid_Linear_Points.push_back(_Mid_Linear_Points.at(_Mid_Linear_Points.size()-1));
    //cout<<"测试 "<<endl;
    //print_vector(Re_Mid_Linear_Points);
   _Mid_Linear_Points.assign(Re_Mid_Linear_Points.begin(), Re_Mid_Linear_Points.end());
}



/*
函数名称 :  get_mid_linear 
调用方式 :  mid_linear = get_mid_linear(save_cols_date);
功能  :  得到每行的轮廓的中心点
说明  :  默认是从左向右寻找 两个点在[Linear_Min Linear_Max]之间的点
        也可以根据需要寻找从右到左的点（当优先考虑右侧的黑线时）
*/
std::vector<int> save_cols_mid_date;
double re_mid_linear = Frame_Width/2.0;//r若检测到小于两个点 则设置为中点值 如果将其放置函数外面则为上一次的保存值

double get_mid_linear(vector<int> filter_colsr)
{
    //cout<<"filter_colsr.size() = "<<filter_colsr.size()<<endl;
    if(filter_colsr.size()> 2){
         //cout<<"filter_colsr.size() = "<<filter_colsr.size()<<endl;
         filter_Mid_Linear_Value(filter_colsr);
         re_mid_linear = Select_Linear_fun(filter_colsr);
        // for(int m = 0;m<filter_colsr.size()-1;m++)
        // {
        //         int Dy_cols = filter_colsr.at(m+1)-filter_colsr.at(m);
        //         if((Dy_cols>Linear_Min)&&(Dy_cols<Linear_Max))
        //         {
        //             re_mid_linear = (filter_colsr.at(m+1)+filter_colsr.at(m))/2;
        //             break;
        //         }
        // }
    }  
    else if(filter_colsr.size()== 2){
            int Dy_cols2 = abs(filter_colsr.at(0)-filter_colsr.at(1));
           // cout<<"Dy_cols2 = "<<Dy_cols2<<" at(0)  = "<<filter_colsr.at(0)<<" at(1) = "<<filter_colsr.at(1)<<endl;
            if((Dy_cols2>Linear_Min)&&(Dy_cols2<Linear_Max))
            {
               re_mid_linear = (filter_colsr.at(0)+filter_colsr.at(1))/2;
            //   cout<<"Dy_cols2 = "<<Dy_cols2<<"  re_mid_linear = "<<re_mid_linear<<endl;

            }
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
void filter_Mid_Linear_Points(vector<Point2d> &_Mid_Linear_Points,int Filter_X_OR_Y)
{
    vector<Point2d> Re_Mid_Linear_Points;
    //print_vector2d(_Mid_Linear_Points);
    for(int i = 1; i < (_Mid_Linear_Points.size()-1);)
     {
          float pre_z = 0;
          float cur_z = 0;
          float fut_z = 0;
          if(Filter_X_OR_Y = 0)
          {
             pre_z  = _Mid_Linear_Points.at(i-1).x;
             cur_z  = _Mid_Linear_Points.at(i).x;
             fut_z  = _Mid_Linear_Points.at(i+1).x;
          }
          else{
             pre_z  = _Mid_Linear_Points.at(i-1).y;
             cur_z  = _Mid_Linear_Points.at(i).y;
             fut_z  = _Mid_Linear_Points.at(i+1).y;
          }
          if((abs(cur_z - pre_z)<Filter_Point_Error_Value)&&(abs(cur_z - fut_z)<Filter_Point_Error_Value))
          {
            Re_Mid_Linear_Points.push_back(_Mid_Linear_Points.at(i));
            i++;
          }  
          else{
              _Mid_Linear_Points.erase(_Mid_Linear_Points.begin()+i);
          }
     }
    //print_vector2d(_Mid_Linear_Points);
   _Mid_Linear_Points.assign(Re_Mid_Linear_Points.begin(), Re_Mid_Linear_Points.end());
}

 void  Index_Rows(cv::Mat frame)
 {
   int rows = frame.rows;
    int cols = frame.cols;
    double mid_linear = frame.cols/2;
    for (int i = 5; i < (rows-5);)
    {  
        uchar* ptr = (uchar*)frame.data +i *cols;
        for (int j = 5; j < (cols - 5); j++)
        {
            int value = ptr[j];
            if(value>0)
            {
              save_cols_date.push_back(j);
            }
        }
        //cout<<"save_cols_date.size() = "<<save_cols_date.size()<<" i = "<<i<<endl;
        //mid_linear = get_mid_linear(save_cols_date);
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
 }

void  Index_Cols_Child_Left(cv::Mat frame,double mid_linear_average)
{
    int rows = frame.rows;
    int cols = frame.cols;
    double mid_linear = frame.cols/2;
    for (int i = mid_linear_average-10; i > 5;)
    {  
        uchar* ptr = (uchar*)frame.data + i;
        for (int j = (rows - 5); j >0; )
        {
            int value = ptr[j*cols];//遍历列元素
            if(value>0)
            {
              save_cols_date.push_back(j);
            }
            j = j-1;
        }
        //cout<<"save_cols_date.size() = "<<save_cols_date.size()<<"  i = "<<i<<endl;
        //print_vector(save_cols_date);
        sort(save_cols_date.begin(),save_cols_date.end());
        mid_linear = get_mid_linear(save_cols_date);
        if(mid_linear!=discard_value)
        {
            pre_mid_linear = mid_linear;
            get_midlinear_point.x = i;//
            get_midlinear_point.y = mid_linear;//
            Mid_Linear_Points.push_back(get_midlinear_point);
        }
        save_cols_date.clear();
        i=i-2;//gap
        ros::spinOnce();  
    }
}

void  Index_Cols_Child_Right(cv::Mat frame,double mid_linear_average)
{
    int rows = frame.rows;
    int cols = frame.cols;
    double mid_linear = frame.cols/2;
    for (int i = mid_linear_average+20; i < (cols-5);)
    {  
        uchar* ptr = (uchar*)frame.data + i;
        for (int j = (rows - 5); j >0; )
        {
            int value = ptr[j*cols];
            if(value>0)
            {
              save_cols_date.push_back(j);
            }
            j = j-1;
        }
        //cout<<"save_cols_date.size()22 = "<<save_cols_date.size()<<" i = "<<i<<endl;
        //print_vector(save_cols_date);
        sort(save_cols_date.begin(),save_cols_date.end());
        mid_linear = get_mid_linear(save_cols_date);
        if(mid_linear!=discard_value)
        {
            pre_mid_linear = mid_linear;
            get_midlinear_point.x = i;//
            get_midlinear_point.y = mid_linear;//
            Mid_Linear_Points.push_back(get_midlinear_point);
        }
        save_cols_date.clear();
        i=i+2;//gap
        ros::spinOnce();  
    }

}

float Get_Few_Linear_Mid(cv::Mat frame)
{
    double mid_linear_few = 0;
    double mid_linear_total = 0;
    int Number_rows = 0;
    double mid_linear = frame.cols/2;
    int rows = frame.rows;
    int cols = frame.cols;
    int Start_Row = 0;
    int End_Row   = 0;
    int Gap_row   = 0;
    if(Select_Linear == 0)//无交叉线时
    {
        Start_Row = rows-1;
        End_Row   = 5;
        Gap_row   = -5;
    }
    else{
         Start_Row = rows-1;
         End_Row   = rows-5;
         Gap_row   = -2;
    }
    
    for (int m = Start_Row; m > End_Row;)
    { 
        uchar* ptr = (uchar*)frame.data +m *cols;
        int re_j = cols/2;
        for (int j = cols/2; j < (cols - 5); j++)
        { 
            re_j -=1;
            int value_R = ptr[j];//向右遍历
            int value_L = ptr[re_j];//向左遍历
            if(value_R>0)save_few_cols_date.push_back(j);
            if(value_L>0)save_few_cols_date.push_back(re_j);
        }
        sort(save_few_cols_date.begin(),save_few_cols_date.end());
        if(Select_Traversal_Mode == Turn_Left)
         reverse(save_few_cols_date.begin(),save_few_cols_date.end());
        mid_linear_few = get_mid_linear(save_few_cols_date);
        //cout<<"mid_linear_few = "<<mid_linear_few<<endl;
        if(mid_linear_few!=discard_value)
        {
            if(Select_Linear == 0)
            {
                pre_mid_linear = mid_linear_few;
                get_midlinear_point.x = mid_linear_few;//
                get_midlinear_point.y =m  ;//
                Mid_Linear_Points.push_back(get_midlinear_point);  
            }
           mid_linear_total = mid_linear_total+mid_linear_few;
           Number_rows++;
        }
        m = m + Gap_row;
        save_few_cols_date.clear();   
    }
    if(Number_rows == 0)
    {
      if(Select_Linear == Turn_Left)
        return (Frame_Width/1 - 10);
      else if(Select_Linear == Turn_Right)
         return Frame_Width/10;
      else
      return Frame_Width/2.0;

    }
    else
    return  mid_linear_total/Number_rows;
}
 void  Index_Cols(cv::Mat frame)
 {
    int rows = frame.rows;
    int cols = frame.cols;
    double mid_linear_average = frame.cols/2;
    mid_linear_average = Get_Few_Linear_Mid(frame);
    //if(mid_linear_average>frame.cols) mid_linear_average = frame.cols/6; 当最初的黑线丢失时
    //cout<<" mid_linear_average222 = "<<mid_linear_average<<endl;

    if(Select_Linear == 2)
    {
         cout<<"mid_linear_average_Right = "<<mid_linear_average<<endl;
         Index_Cols_Child_Right(frame,mid_linear_average);
    }
    else if(Select_Linear == 1)
    {
      cout<<"mid_linear_average_Left = "<<mid_linear_average<<endl;
      Index_Cols_Child_Left(frame,mid_linear_average);
    }
    else{
        
    }
 }

int Recording_Num_Self_add = 0;
int Pre_Recording_Num      = 0;
void OverTurn(int Recording_Num)
{
   if((Recording_Num>15)&&(Pre_Recording_Num != Recording_Num))
   {
       Recording_Num_Self_add++;
   }
   if((Recording_Num_Self_add>50)&&(Recording_Num<8))
   {
       Select_Linear = 0;
       Recording_Num_Self_add = 0;
   }
   Pre_Recording_Num = Recording_Num;
}
/*
函数名称 :  Fine_Center_Line 
调用方式 :  Fine_Center_Line(frame);
功能  :    巡线程序总入口
说明  :  

*/

 void vision_processing::Fine_Center_Line(cv::Mat frame)
 { 
    Canny(frame, frame, 3, 9, 3);
    cv::cvtColor(frame, img_rgb, CV_GRAY2RGB);
    //Index_Rows(frame);
    Index_Cols(frame);
    cout<<"Rec_add = "<<Recording_Num_Self_add<<" Select_Linear = "<<Select_Linear<<endl;
    //if(Select_Linear != 0)
    //   OverTurn(Mid_Linear_Points.size());
    cout<<"Mid_Linear_Points.size() = "<<Mid_Linear_Points.size()<<endl;
    if(Mid_Linear_Points.size()>2) //开头几行出错 可能会导致整体出错
    {
        //Select_Linear == 0 表明遍历行 对行进行滤波
        //！= 0 则遍历列 则对列进行滤波 
        if(Select_Linear == 0)filter_Mid_Linear_Points(Mid_Linear_Points,Filter_X);
         else filter_Mid_Linear_Points(Mid_Linear_Points,Filter_Y);
    }
    cout<<"Mid_Linear_Points.size() = "<<Mid_Linear_Points.size()<<endl;
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
    //先publish 再clear
    Send_Linear_Point.publish(_Mid_Linear_Points);
    Mid_Linear_Points.clear();
    //cv::cvtColor(frame, img_rgb, CV_GRAY2RGB);
    //cv::circle(img_rgb,cv::Point2i(frame.cols/2,frame.rows/2),3,cv::Scalar(255,0,0),-1,2);
    cv::line(img_rgb,cv::Point2i(frame.cols/2,0),cv::Point2i(frame.cols/2,frame.rows), cv::Scalar(0,0,100), 2,CV_AA ); 
    //mono8 bgr8
    Publisher_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_rgb).toImageMsg();  
    Publisher_Image.publish(Publisher_msg); 
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
    //vision_processing VPF1;
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


/*
函数名      ：fit_linear_fun_experiment
功能       ：测试线性拟合函数
调用方式    ：
说明       ：以图片的形式打印出拟合的结果
*/
void vision_processing::fit_linear_fun_experiment()
{
  //创建一个用于绘制图像的空白图
	cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
	//输入拟合点
	std::vector<cv::Point> points;
	points.push_back(cv::Point(48, 58));
	points.push_back(cv::Point(105, 98));
	points.push_back(cv::Point(155, 160));
	points.push_back(cv::Point(212, 220));
	points.push_back(cv::Point(248, 260));
	points.push_back(cv::Point(320, 300));
	points.push_back(cv::Point(350, 360));
	points.push_back(cv::Point(412, 400));
 
	//将拟合点绘制到空白图上
	for (int i = 0; i < points.size(); i++)
	{
		cv::circle(image, points[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
	}
 
	cv::Vec4f line_para; 
	cv::fitLine(points, line_para,CV_DIST_L2, 0, 1e-2, 1e-2);
 
	std::cout << "line_para = " << line_para << std::endl;
 
	//获取点斜式的点和斜率
	cv::Point point0;
	point0.x = line_para[2];
	point0.y = line_para[3];
 
	double k = line_para[1] / line_para[0];
 
	//计算直线的端点(y = k(x - x0) + y0)
	cv::Point point1, point2;
	point1.x = 0;
	point1.y = k * (0 - point0.x) + point0.y;
	point2.x = 640;
	point2.y = k * (640 - point0.x) + point0.y;
	cv::line(image, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);
	cv::imshow("image", image);
	cv::waitKey(10);
}

void vision_processing::Draw_Line(std::vector<cv::Point> Fit_Points,cv::Vec4f _line_para)
{
        cv::Mat image1 = cv::Mat::zeros(Frame_Height, 160, CV_8UC3);
        cv::Vec4f line_para; 
        for (int i = 0; i < Fit_Points.size(); i++)
        {
            cv::circle(image1, Fit_Points[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
        }
        double cos_theta = _line_para[0];
        double sin_theta = _line_para[1];
        cv::Point point0;
        point0.x = _line_para[2];
        point0.y = _line_para[3];
        double D_K = sin_theta / cos_theta;
        if(D_K>50) D_K = 50;
        if(D_K<-50) D_K = -50;
        cv::Point point1, point2;
        point1.x = 1;//fit_line_points[fit_line_points.size()/8].x
        point1.y = D_K * (point1.x - point0.x) + point0.y;
        point2.x = 110;//fit_line_points[fit_line_points.size()].x
        point2.y = D_K * (point2.x - point0.x) + point0.y;
        cv::line(image1, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);
        Publisher_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();  
        Publisher_Fit_Image.publish(Publisher_msg); 
        cv::imshow("image1", image1);
}

/*
函数名称 :  fit_lnear 
调用方式 :  cout<<"K = "<<VP_Control.fit_lnear(fit_line_points)<<endl;
功能  :    根据图像处理得到的线中心坐标点 进行拟合 得到斜率K值
说明  :    返回K值
*/
double Line_K = 0;
double vision_processing:: fit_lnear(std::vector<cv::Point> Fit_Points)
{
     if(Fit_Points.size()>2)
     {
            cv::Vec4f line_para; 
            cv::fitLine(Fit_Points, line_para,CV_DIST_L2, 0, 1e-2, 1e-2);
            double cos_theta = line_para[0];
            double sin_theta = line_para[1];
            cv::Point point0;
            point0.x = line_para[2];
            point0.y = line_para[3];
            Line_K = sin_theta / cos_theta;
            if(Line_K>50) Line_K = 50;
            if(Line_K<-50) Line_K = -50;
            Line_K = -(1/Line_K)*50;//小于45 时 Line_K<1
            if(Line_K>80) Line_K = 80;
            if(Line_K<-80) Line_K = 80-50;
            double b = point0.y - Line_K*point0.x;
            Draw_Line(Fit_Points,line_para);
     }
     return Line_K;

}
 


/*
函数名称 :  Get_Deviation_MidLnear 
调用方式 :   
功能  :     
说明  :    返回K值
*/
float Total_Value    = 0;
double vision_processing:: Get_Deviation_MidLnear(std::vector<cv::Point> Fit_Points)
{
     int Point_Size = Fit_Points.size();
     int Piont_Number     = 0; 
     if(Fit_Points.size()>2)
     {
        // for(int i = 2;i<Point_Size;)
        // {
        //     Total_Value += (Fit_Points[i].x - Frame_Width/2.0);
        //     //cout<<"x = "<< Fit_Points[i].x<<" Total_Value = "<<Total_Value<<endl;
        //     i += 3;
        //     Piont_Number++;
        // }
        // Total_Value = Total_Value/Piont_Number;
        //Total_Value =  (Fit_Points[Point_Size-1].x - Frame_Width/2.0);
        float Mean_Mid_value = (Fit_Points[Fit_Points.size()/2+1].x+Fit_Points[Fit_Points.size()/2].x+Fit_Points[Fit_Points.size()/2-1].x)/3;
        Total_Value = Fit_Points[2].x*0.0+Mean_Mid_value*1+Fit_Points[Fit_Points.size()-2].x*0.0;
        Total_Value = Total_Value- Frame_Width/2.0;
        cout<<"Fit_Points.size() = "<<Point_Size<<endl;
     }
     return Total_Value;
}


double Dev_Line_Value = 0;
//float Total_Value    = 0;
double vision_processing:: Get_Deviation_Lnear(std::vector<cv::Point> Fit_Points)
{
     int Point_Size = Fit_Points.size();
     int Piont_Number     = 0; 
     float All_Date       = 0;
     cout<<"Get_Deviation_Lnear"<<endl;
     if(Fit_Points.size()>2)
     {
        for(int i = 0;i<Point_Size;i++)
        {
            All_Date += Fit_Points[i].x;
            //cout<<"Fit_Points[i].x = "<<Fit_Points[i].x<<endl;
            Piont_Number++;
        }
        //cout<<"All_Date = "<<All_Date<<"  Piont_Number = "<<Piont_Number<<endl;
        float Average_Value = All_Date/Piont_Number;
        //cout<<"Average_Value = "<<Average_Value<<endl;
        for(int i = Point_Size/2;i<Point_Size-1;)
        {
            Dev_Line_Value +=  abs(Fit_Points[i].x -Average_Value);
            //cout<<"x = "<< Fit_Points[i].x<<" Total_Value = "<<Total_Value<<endl;
            i += 2;
            Piont_Number++;
        }
        Dev_Line_Value = Dev_Line_Value/Piont_Number;
        //Total_Value =  (Fit_Points[Point_Size-1].x - Frame_Width/2.0);
        //cout<<"Fit_Points.size() = "<<Point_Size<<endl;
     }
     return Dev_Line_Value;
}

void vision_processing:: Send_speed(float speed ,float adjust)
{
    geometry_msgs::Twist twist1;
    twist1.linear.x = speed; twist1.linear.y = 0; twist1.linear.z = 0;
    twist1.angular.x = 0; twist1.angular.y = 0; twist1.angular.z = adjust/0.2136;//angle_adjust_linear
    Send_Request_Speed.publish(twist1);
    cout<<"adjust = "<<adjust<<endl;
}
void vision_processing::print_amcl_linear_x_ahead1()
{
  cout<<"Hello SLAM amcl_linear_x_ahead"<<endl;
}
