#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/legacy/compat.hpp>
#include <cv_bridge/cv_bridge.h>
#include "mini_robot_control/vision_function.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>

//#include "vision_function.h"

#include <vector>
#include <stdio.h>
using namespace cv;
using namespace std;
//阈值回调函数
 void on_Threshold(int, void*)
 {
    // threshold(Split_S_img, Split_S_img, g_nThresholdValue_GARY, 255, THRESH_BINARY_INV);
 }

/*
void drawGrayImage(void)
{
	IplImage *img=cvLoadImage("lena.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (img == NULL)
		exit(0);
	int dims=1; //创建一维直方图
	int sizes[]={256}; //共有256个取值范围
	int type=CV_HIST_ARRAY; //表示使用密集多维矩阵结构
	float range[]={0, 255}; //取值范围为0-255
	float *ranges[]={range}; 
	CvHistogram *hist=NULL;//创建直方图的空指针结构
 
	hist=cvCreateHist(dims, sizes, type, ranges, 1);//创建直方图
	cvCalcHist(&img, hist, 0, NULL); //统计直方图 计算图像直方图的函数
	cvNormalizeHist(hist, 1.0); //归一化直方图
 
	int hist_height=256; //直方图高度
	int hist_size=256;   //直方图尺寸
	int scale=2;
	//创建一张一维直方图的“图”，横坐标为灰度级，纵坐标为像素个数（*scale） 彩色图像
	IplImage *hist_image=cvCreateImage(cvSize((hist_size*scale), hist_height), IPL_DEPTH_8U, 3); 
	if (hist_image == NULL)
		exit(0);
	cvZero(hist_image);
 
	float max=0; //直方图中的最大值
	cvGetMinMaxHistValue(hist, 0, &max, NULL, NULL);

	for (int i=0; i<hist_size; i++)
	{
		float val=cvQueryHistValue_1D(hist, i);
		int intensity=cvRound(hist_height*val/max);
		cvRectangle(hist_image, cvPoint(i*scale, hist_height-1), cvPoint((i+1)*scale-1, hist_height-intensity-1), CV_RGB(255, 255, 255), -1, 8, 0);
	}
 
	cvNamedWindow("img");
	cvNamedWindow("hist_image");
	cvShowImage("img", img);
	cvShowImage("hist_image", hist_image);
	cvWaitKey(0);
	cvDestroyAllWindows();
	cvReleaseImage(&img);
	cvReleaseImage(&hist_image);
}
*/


typedef Point_<double> Point2d;
sensor_msgs::PointCloud g_Set_Points;
Point2d get_midlinear_poin1;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
   // ros::Publisher send_Linear_Point1 = nh.advertise<sensor_msgs::PointCloud>("Linear_Point1",1);//改为1    
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    //ros::Publisher send_Linear_Point1 = nh.advertise<sensor_msgs::PointCloud>("Linear_Point1",1);//改为1
   
    cv::VideoCapture cap(1);//1 0s

    cap.set(CV_CAP_PROP_FRAME_WIDTH,140);//宽度 320
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,180);//高度240
    cap.set(CV_CAP_PROP_FPS, 120);///30
    cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 1);
    printf("width = %.2f\n",cap.get(CV_CAP_PROP_FRAME_WIDTH));
    printf("height = %.2f\n",cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    printf("fbs = %.2f\n",cap.get(CV_CAP_PROP_FPS));
    printf("brightness = %.2f\n",cap.get(CV_CAP_PROP_BRIGHTNESS));
    printf("contrast = %.2f\n",cap.get(CV_CAP_PROP_CONTRAST));
    printf("saturation = %.2f\n",cap.get(CV_CAP_PROP_SATURATION));
    printf("hue = %.2f\n",cap.get(CV_CAP_PROP_HUE));
    printf("exposure = %.2f\n",cap.get(CV_CAP_PROP_EXPOSURE));

    if (!cap.isOpened()) {
                ROS_INFO("cannot open video device\n");
                return 1;
     }
    cv::Mat frame= Mat(120, 160, CV_8UC3);;
    cv::Mat frame_gray= Mat(120, 160,CV_8UC1);
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(100);//以10ms间隔发送图片
    string ShowName="current_video";
    namedWindow(ShowName, 1 );
    createTrackbar("parameter", ShowName, &g_nThresholdValue_GARY, 255, on_Threshold);
    vision_processing VPF;
   
    //drawGrayImage();
     while(ros::ok())
        {
        ros::spinOnce();
        cap >> frame;  
        if (!frame.empty()) {  
            //bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序
            //mono8：CV_8UC1， 灰度图像
            //OpenCV图像转换为ROS消息的函数
            //Send_stop();
            resize(frame, frame, frame_gray.size());
            cv::imshow("BGR",frame);
            GaussianBlur(frame,frame,Size(9,9),0,0);
            cv::medianBlur(frame,frame,5);
            VPF.Deal_gray_Vision(frame);
            //VPF.Deal_HSV_Vision (frame);
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame).toImageMsg();  
            pub.publish(msg); 
            circle(frame,cv::Point2i(frame.cols/2,frame.rows/2),3,cv::Scalar(255,0,0),-1,8);
            cv::imshow(ShowName,frame);
            if(waitKey(20) >=0) 
             break;
        }
        ROS_INFO("runnning!");
        ros::spinOnce();  
        loop_rate.sleep();//与ros::Rate loop_rate相对应,休息10ms
    } 
    return 0;
}
