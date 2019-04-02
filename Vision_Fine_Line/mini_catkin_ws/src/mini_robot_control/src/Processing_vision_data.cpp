#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <stdio.h>
using namespace cv;
using namespace std;
vector<Mat> HSVChannels;
cv::Mat Split_S_img= Mat(120, 160, CV_8UC1);
int g_nThresholdValue_GARY = 180;
//阈值回调函数

void on_Threshold(int, void*)
{
    threshold(Split_S_img, Split_S_img, g_nThresholdValue_GARY, 255, THRESH_BINARY_INV);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    cv::VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 240);//宽度 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 320);//高度
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
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(10);//以10ms间隔发送图片
    string ShowName="current_video";
    namedWindow(ShowName, 1 );
    createTrackbar("参数值", ShowName, &g_nThresholdValue_GARY, 255, on_Threshold);
    while (nh.ok()) {
        cap >> frame;  
        if (!frame.empty()) {  
            //bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序
            //mono8：CV_8UC1， 灰度图像
            //OpenCV图像转换为ROS消息的函数
            cvtColor(frame, frame, CV_BGR2HSV);
            split(frame, HSVChannels); //分离通道
            resize(HSVChannels.at(1), Split_S_img, Split_S_img.size());
            GaussianBlur(Split_S_img,Split_S_img,Size(5,5),0,0);
            on_Threshold(0, 0);
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", Split_S_img).toImageMsg();  
            pub.publish(msg);  
            cv::imshow(ShowName,Split_S_img);
            if(waitKey(30) >=0) 
             break;
        }
        ROS_INFO("runnning!");
        ros::spinOnce();  
        loop_rate.sleep();//与ros::Rate loop_rate相对应,休息10ms
    }
}
