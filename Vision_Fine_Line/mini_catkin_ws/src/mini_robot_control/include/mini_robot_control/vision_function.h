#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/legacy/compat.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
extern int g_nThresholdValue_GARY;
class vision_processing
{
  public:
    void Read_Picture_W_Circle();//cv::Mat img
    void Deal_gray_Vision(cv::Mat frame);
    void Deal_HSV_Vision(cv::Mat frame);
    void print_amcl_linear_x_ahead1();
 private:

};

