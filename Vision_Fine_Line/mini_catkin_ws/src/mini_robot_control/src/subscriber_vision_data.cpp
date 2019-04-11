#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/PointCloud.h>
using namespace cv;
using namespace std;
std::vector<cv::Point> Creat_vector_point;
void Creat_vector_pointfun()
{
  // for(int i = 0;i<120;)
  // {
  //  	Creat_vector_point.push_back(cv::Point(i, i));
  //   i = i+5;
  // }

    for(int i = 120;i>0;)
    {
      Creat_vector_point.push_back(cv::Point(i, -i));
      i = i-5;
     }
	// Creat_vector_point.push_back(cv::Point(105, 98));
	// Creat_vector_point.push_back(cv::Point(155, 160));
	// Creat_vector_point.push_back(cv::Point(212, 220));
	// Creat_vector_point.push_back(cv::Point(248, 260));
	// Creat_vector_point.push_back(cv::Point(320, 300));
	// Creat_vector_point.push_back(cv::Point(350, 360));
	// Creat_vector_point.push_back(cv::Point(412, 400));
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

  cv::imshow("view", img_rgb);  
  // fail if don't have waitKey(3).
  cv::waitKey(3);
}  

sensor_msgs::PointCloud g_Set_Points;
vector<Point> fit_line_points;
void Get_Linear_Pose(const sensor_msgs::PointCloud& Points) 
{
  //int Point_size = Points.points.size();
  //1fit_line_points.resize(Point_size);
  fit_line_points.clear();
  for(unsigned int i = 0; i < Points.points.size(); i++)
  {
     //cout<<Points.points[i].x<<"  ";
     fit_line_points.push_back(cv::Point(Points.points[i].x, Points.points[i].y));
  }
}

void fit_lnear(std::vector<Point> Fit_Points)
{
     cv::Mat image1 = cv::Mat::zeros(120, 160, CV_8UC3);
     cv::Vec4f line_para; 
     for (int i = 0; i < Fit_Points.size(); i++)
      {
         cv::circle(image1, Fit_Points[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
      }
      cv::fitLine(Fit_Points, line_para,CV_DIST_L2, 0, 1e-2, 1e-2);
      double cos_theta = line_para[0];
      double sin_theta = line_para[1];
      cv::Point point0;
      point0.x = line_para[2];
      point0.y = line_para[3];
      double k = sin_theta / cos_theta;
      cv::Point point1, point2;
      point1.x = 1;//fit_line_points[fit_line_points.size()/8].x
      point1.y = k * (point1.x - point0.x) + point0.y;
      point2.x =  110;//fit_line_points[fit_line_points.size()].x
      point2.y = k * (point2.x - point0.x) + point0.y;
      cv::line(image1, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);
      cv::imshow("image1", image1);
      ROS_INFO("runnning!");
      cout<<"line_para = "<<line_para<<" k = "<<k<<endl;
}
 // 
  // if(Point_size>1)
  // {
       
	// //将拟合点绘制到空白图上
  //     for (int i = 0; i < fit_line_points.size(); i++)
  //     {
  //       cv::circle(image1, fit_line_points[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
      
  //     }

  //     	cv::Vec4f line_para; 
  //     cv::fitLine(fit_line_points, line_para,CV_DIST_L2, 0, 1e-2, 1e-2);
    
  //     std::cout << "line_para = " << line_para << std::endl;
    
  //     //获取点斜式的点和斜率
  //     cv::Point point0;
  //     point0.x = line_para[2];
  //     point0.y = line_para[3];
  //     double cos_theta = line_para[0];
  //     double sin_theta = line_para[1];
  //     double k = sin_theta / cos_theta;
  //     double b =  point0.y - k * point0.x;
  //     std::cout << " k = " <<  k<<" b = " <<b<< std::endl;

  //     //计算直线的端点(y = k(x - x0) + y0)
  //     cv::Point point1, point2;
  //     point1.x = fit_line_points[fit_line_points.size()/3].x;
  //     point1.y = k * (point1.x - point0.x) + point0.y;
  //     point2.x =  fit_line_points[fit_line_points.size()/2].x;
  //     point2.y = k * (point2.x - point0.x) + point0.y;
  //     cv::line(image1, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);
  //     cv::imshow("image1", image1);
     // cv::waitKey(0);
      /*
      cv::Vec4f line;
      cv::fitLine(fit_line_points,
                    line,
                    CV_DIST_L2   ,
                    0,
                    0.01,
                    0.01);
      double cos_theta = line[0];
      double sin_theta = line[1];
      double x0 = line[2], y0 = line[3];
      double k = sin_theta / cos_theta;
      double b = y0 - k * x0;
      double x = 0;
      double y = k * x + b;
      cout <<"k="<< k << endl;//the gradient pf "k" is equal to line[1]/line[0]
      cout <<"b="<< b << endl;
      cout<<"line = "<<line<<endl;
      */
  //}



int fit_linear_fun()
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
	cv::waitKey(0);
	return 0 ;
}
int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "image_subscriber");  
  ros::NodeHandle nh;  
  //然后打开一个图像显示窗口,并单独安排一个线程显示图像 
  //cv::namedWindow("view", CV_WINDOW_NORMAL);   
  cv::startWindowThread();  
  image_transport::ImageTransport it(nh);  
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::Subscriber Get_Pose = nh.subscribe("Linear_Point", 1000, Get_Linear_Pose);
  //fit_linear_fun();
  //Creat_vector_pointfun();
  //fit_lnear(Creat_vector_point);
 // Creat_vector_point.clear();
  while(ros::ok()){
    if(fit_line_points.size()>2)
      fit_lnear(fit_line_points);
    // fit_line_points.clear();
     ros::spinOnce();
    if(waitKey(20) >=0) 
      break;
  }
  ros::spin();  //等待message的到来
  cv::destroyWindow("view");  
  return 0;
} 
