#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include "custom_msg_topic/custom_msg.h"
#include <turtlesim/Spawn.h>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>
#include "mini_robot_control/trajectory_plannin.h"
#include "mini_robot_control/amcl_linear_x_ahead.h"
#include <mini_robot_control/Velocities.h>
#include <fstream>
#include <string>
#include <algorithm>  
#include <cstdlib>
# include <stdio.h>
#include "string.h"
#include <opencv2/opencv.hpp>
//#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

int test_pp = 0;
int state = 0;
float Classification_group[10];
float distan_Laser[360];
float slow_down_laser[360];
float Speed_change = 0;
float Current_speed = 0;
float get_max_obstacle_x = 0;
int   Obstacle_amount = 0;
int   Obstacle_index_through = 0;
int   Obstacle_index_stop = 0;
int get_obstacle_point_fun_parameter_init = 0;
float pre_f_obstacle_y = 0;
int  obstacle_y_positive_num = 0;
int  obstacle_y_negative_num = 0; 
int  state_one_flag = 0;
int  Continued = 1;
#define One_Area_Size               45
#define Divided_Amount              180/One_Area_Size
#define No_Obstacles                255
#define robot_L                     0.51
#define Default_value               1024
#define Aisle_width_half            0.9
#define lidar_gap                   1//2
#define Robot_width_big             0.51
#define Error_control_vaule         2//3
#define Error_control_y_vaule       0.1
#define obstacle_limit_num          4
#define _min_lidar_date              0.3
const float ratation_value = 0.1;//0.2
const float P_i = 3.1415926;
const float stop_rel_point_x      = 0.65;//0.65 1.25
const float stop_rel_point_y      = 0.29;
const float Robot_Boundary        = 0.68;//0.65 0.7
float obstacle_x = 0;
float obstacle_y = 0;
float get_max_value = 0;
float get_min_value = Default_value;
int test = 0;
float W_pre_date[271][3]={0};
int   two_arrary_num      = 0;
int flag = 0; 
//float W_pre_date_2[271]={0};
//int Obstacle_block_coordinates[10];
//int Obstacle_block_num = 0;//
typedef Point_<double> Point2d;
typedef Point3_<double> Point3d;
std::vector<int>  save_date;
std::vector<int>  diff_point3_max_y;
std::vector<int>  Stop_selecting_max;
vector<Point3d> _whole_obstacles_point3;
Point2d get_obstacle_point;
Point3d whole_point3;
Point3d boundary_point3;
Point3d get_near_Obstacle_point;
float get_max_value_fun(float Settings);
float According_to_obstacles_chance_speed(vector<Point3d> obstacles_point3,int pass_index,int stop_index);
typedef struct Result_pose{
    float x;
    float y;
    float angle;
}Result_pose;
Result_pose robot_pose;
Result_pose relative_obstacle_point3;
/*
函数名   ：Result_pose return_amcl_get_pose
功能     ：返回amcl的位姿参数
调用方式  ：Result_pose robot_pose = return_amcl_get_pose();
说明     ：缺点： 每次都初始化 运行过程缓慢  优点：形式简明
*/
Result_pose return_amcl_get_pose(){
    Result_pose ret;
    tf::TransformListener listener_amcl_pose;
    tf::StampedTransform transform_amcl_pose;
    Eigen::Quaterniond q_amcl;
    ros::Time begin = ros::Time::now();
    listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
    listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
    q_amcl.x() = transform_amcl_pose.getRotation().getX();
    q_amcl.y() = transform_amcl_pose.getRotation().getY();
    q_amcl.z() = transform_amcl_pose.getRotation().getZ();
    q_amcl.w() = transform_amcl_pose.getRotation().getW();
    Eigen::Vector3d euler_linear = q_amcl.toRotationMatrix().eulerAngles(0, 1, 2);
    //cout << "  A = "<<euler_linear[2]*57.29578<<endl;
    ros::spinOnce();
   // cout << "  amcl_W_x11 = "<<transform_amcl_pose.getOrigin().x() << "  W_y = "<<transform_amcl_pose.getOrigin().y()<< "  A = "<<euler_linear[2]*57.29578<<endl ;
    ret.x = transform_amcl_pose.getOrigin().x();
    ret.y = transform_amcl_pose.getOrigin().y();
    ret.angle = euler_linear[2];//*57.29578
    return ret;
}
class Function
{
   public:
      void  get_relative_obstacle_point3( std::vector<float> get_distance_angle,int i);
      void  print_vector(vector<Point3d> point3);
      void  print_array(float array[]);
      float get_relative_control_point(vector<Point3d> point3);
       int  get_through_index(vector<Point3d> whole_obstacles_point3);
       int  get_stop_index(vector<Point3d> whole_obstacles_point3);
      void  get_diff_point_fun(vector<Point3d> whole_obstacles_point3,float *diff_point3_y);
      void  get_max_index(float array[]);
      Result_pose get_min_x_y(vector<Point3d> point3);
     // Function();  // 这是构造函数
   private:
      float x ;
      float y ;
      float pre_y;
      float dy;
};
/*
函数名：get_relative_obstacle_point3
参数  :get_distance_angle:传入激光雷达的距离容器 i:雷达数据所对应的位置
功能  ：等到障碍物的相对坐标
说明  ：
*/
vector<Point3d> _Relative_point3;//
void Function::get_relative_obstacle_point3(std::vector<float> get_distance_angle,int i)
{
      //y = get_distance_angle[i]*cos(((i-90)/180.0)*P_i);//上x 右Y
      //x = get_distance_angle[i]*sin(((i-90)/180.0)*P_i);

      y = get_distance_angle[i]*cos(((270-i)/180.0)*P_i);//上x 左Y 符合右手定则
      x = get_distance_angle[i]*sin(((270-i)/180.0)*P_i);
    // cout<<"obstacle_x = "<<obstacle_x<<" obstacle_y = "<<obstacle_y<<endl;
      if(abs(y)<0.6)//0.38
      {
//        cout<<"测试1"<<endl; 
      //  cout<<"x = "<<x<<"  i = "<<i<<"  get_distance_angle[i] = "<<get_distance_angle[i]<<endl;
        get_near_Obstacle_point.x = x;
        get_near_Obstacle_point.y = y;
        get_near_Obstacle_point.z = i;
        _Relative_point3.push_back(get_near_Obstacle_point);
      }
//      cout<<"Function x = "<<x<<" y = "<<y<<endl;
}

void Function:: print_vector(vector<Point3d> point3)
{
   //  cout<<"相对障碍物坐标"<<endl;
     for(int i = 0; i < point3.size(); i++)
     {
         //cout<<" x = "<<point3.at(i).x<<" y = "<<point3.at(i).y<<" z = "<<point3.at(i).z<<endl;
         oFile_init<<" x = "<<point3.at(i).x<<" y = "<<point3.at(i).y<<" z = "<<point3.at(i).z<<endl;
     }
}

Result_pose Function:: get_min_x_y(vector<Point3d> point3)
{
//     cout<<"得到最小的X 和 Y"<<endl;
     Result_pose min_pose;
     float min_x = Default_value;
     float min_y = Default_value;
     for(int i = 0; i < point3.size(); i++)
     {
       //寻找最近X 和点对应的Y点
        if(min_x>abs(point3.at(i).x))
        {
           min_x = abs(point3.at(i).x);
           min_y = abs(point3.at(i).y);
        }
         
       // if(min_y>abs(point3.at(i).y))
       //  min_y = abs(point3.at(i).y);
     }
     min_pose.x = min_x;
     min_pose.y = min_y;
     //cout<<"get_min_x_y min_x = "<<min_x<<"get_min_x_y min_y = "<<min_y<<endl;
     return min_pose;
}

void Function:: print_array(float array[])
{
  for(int i = 0;array[i]!='\0';i++)
  {
     cout<<" i = "<<i<<" array[i] = "<<array[i]<<endl;
  }
}
void Function::get_max_index(float array[])
{
     int num = 0;
     for(int i = 0;array[i]!='\0';i++) //获取数组的大小待优化
     {
         num = i;
     }
     float max = *max_element(array, array + (num-1));
     int max_index = distance(array, max_element(array, array + (num-1)));
}

float Function::get_relative_control_point(vector<Point3d> point3)
{
   Function obstacle_fun;
   if( point3.size()>0)
   {
     
    // obstacle_fun.print_vector(_Relative_point3);
     Result_pose min_pose = obstacle_fun.get_min_x_y(_Relative_point3);
     int num = point3.size()/2;
     relative_obstacle_point3.x = min_pose.x;//point3.at(num).x
     relative_obstacle_point3.y =  point3.at(num).y;//min_pose.y
  //   cout<<"min_pose min_x = "<<min_pose.x<<"   relative_obstacle_point3.x = "<<relative_obstacle_point3.x<<endl;
     return point3.at(num).y;
    //   return min_pose.y;
   }
   else
   {
     relative_obstacle_point3.x = Default_value;
     relative_obstacle_point3.y = Default_value;
     return 0;
   } 
   
}


/*
函数名 ：filter_mutation_z
参数  :传入whole_obstacles_point3的地址 同时修改其值
功能  ：对容器whole_obstacles_point3 进行滤波
说明  ：例如出现的角度为1023 151 176 177 178  滤掉151 因为除头尾外障碍物角度应该有一定的连续性
*/

void filter_mutation_z(vector<Point3d> &whole_obstacles_point3)
{
    Function obstacle_fun;
    vector<Point3d> _whole_obstacles_point3;
//   obstacle_fun.print_vector(whole_obstacles_point3);
   _whole_obstacles_point3.push_back(whole_obstacles_point3.at(0));
    for(int i = 1; i < (whole_obstacles_point3.size()-1);i++)
     {
    //    cout<<" x = "<<whole_obstacles_point3.at(i).x<<" y = "<<whole_obstacles_point3.at(i).y<<" z = "<<whole_obstacles_point3.at(i).z<<endl;
          float pre_z  = whole_obstacles_point3.at(i-1).z;
          float cur_z  = whole_obstacles_point3.at(i).z;
          float fut_z  = whole_obstacles_point3.at(i+1).z;
          if((abs(cur_z - pre_z)<Error_control_vaule)||(abs(cur_z - fut_z)<Error_control_vaule))
          {
            _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
          }  
     }
   _whole_obstacles_point3.push_back(whole_obstacles_point3.at(whole_obstacles_point3.size()-1));
   whole_obstacles_point3.assign(_whole_obstacles_point3.begin(), _whole_obstacles_point3.end());
  // obstacle_fun.print_vector(_whole_obstacles_point3);
}


/*
函数名 ：filter_mutation_y
参数  :传入whole_obstacles_point3的地址 同时修改其值
功能  ：对容器whole_obstacles_point3中 同个障碍物不连续的y 进行滤波
说明  ：
*/

void filter_mutation_y(vector<Point3d> &whole_obstacles_point3)
{
    Function obstacle_fun;
    vector<Point3d> _whole_obstacles_point3;
    _whole_obstacles_point3.push_back(whole_obstacles_point3.at(0));
    for(int i = 1; i < (whole_obstacles_point3.size()-1);i++)
     {
          if( i == 1)
          {
              float cur_y  = whole_obstacles_point3.at(i).y;
              float fut_y  = whole_obstacles_point3.at(i+1).y;
              
              if(abs(cur_y - fut_y)<Error_control_y_vaule)
                {
                  _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
                }
          }
          else if(i == (whole_obstacles_point3.size()-2))
          {
                float pre_y  = whole_obstacles_point3.at(i-1).y;
                float cur_y  = whole_obstacles_point3.at(i).y;
                if(abs(cur_y - pre_y)<Error_control_y_vaule)
                {
                  _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
                }
          }
          else
          {
                float pre_y  = whole_obstacles_point3.at(i-1).y;
                float cur_y  = whole_obstacles_point3.at(i).y;
                float fut_y  = whole_obstacles_point3.at(i+1).y;
                if((abs(cur_y - pre_y)<Error_control_y_vaule)||(abs(cur_y - fut_y)<Error_control_y_vaule))
                {
                  _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
                }  
          } 
     }
   _whole_obstacles_point3.push_back(whole_obstacles_point3.at(whole_obstacles_point3.size()-1));
   whole_obstacles_point3.assign(_whole_obstacles_point3.begin(), _whole_obstacles_point3.end());
}


/*
函数名 ：filter_max_y
参数  :传入whole_obstacles_point3的地址 同时修改其值
功能  ：对容器whole_obstacles_point3中 边界异常出现的值 异常值个数小于等于3个时有效 进行滤波
说明  ：
*/

void filter_max_y(vector<Point3d> &whole_obstacles_point3)
{
    Function obstacle_fun;
    vector<Point3d> _whole_obstacles_point3;
//   obstacle_fun.print_vector(whole_obstacles_point3);
   if(whole_obstacles_point3.size()<6)
   {
     _whole_obstacles_point3.push_back(whole_obstacles_point3.at(0));
     for(int i = 1; i < (whole_obstacles_point3.size()-1);i++)
     {
          float cur_z  = whole_obstacles_point3.at(i).y;
          if((abs(cur_z - robot_pose.y)<0.8))
          {
            _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
          }  
     }
     _whole_obstacles_point3.push_back(whole_obstacles_point3.at(whole_obstacles_point3.size()-1));
     whole_obstacles_point3.assign(_whole_obstacles_point3.begin(), _whole_obstacles_point3.end());
   }

}
void Function::get_diff_point_fun(vector<Point3d> whole_obstacles_point3,float *diff_point3_y)
{
    cout<<"whole_obstacles_point3.size() = "<<whole_obstacles_point3.size()<<endl;
   // oFile_init<<"whole_obstacles_point3.size() = "<<whole_obstacles_point3.size()<<endl;
    for(int i = 0; i < (whole_obstacles_point3.size()-1); i++)
     {
         if((i>0)&&(i<(whole_obstacles_point3.size()-1)))
         {
           get_max_obstacle_x = get_max_value_fun(whole_obstacles_point3.at(i).x);
         }
          pre_y  = whole_obstacles_point3.at(i).y;
          dy = abs(whole_obstacles_point3.at(i+1).y - pre_y);
        //  cout<<" x = "<<whole_obstacles_point3.at(i).x<<" y = "<<whole_obstacles_point3.at(i).y<<" z = "<<whole_obstacles_point3.at(i).z<<endl;
//          oFile_init<< "dy"<< " "<<dy<<" "<<"y"<<" "<<whole_obstacles_point3.at(i).y<< " "<<"x"<<" "<<whole_obstacles_point3.at(i).x<<" "<< " i"<< " "<<whole_obstacles_point3.at(i).z<<endl;
          diff_point3_y[i] = dy;
     }
//     oFile_init<< "next   next   next"<< " "<<endl;
    
}


/*
函数名 get_through_index
参数  :传入可以通过障碍物的坐标点 得到一个离机器人y轴最近的点
功能  ：机器人选择离自身较近的可通过的空
说明  ：
*/

int Function::get_through_index(vector<Point3d> whole_obstacles_point3)
{
    float mid_point_y = 255;
    int Obstacle_index_through    = Default_value;//255
    for(int i = 0;i<diff_point3_max_y.size();i++)
    {
        if(mid_point_y>abs(robot_pose.y-(whole_obstacles_point3.at(diff_point3_max_y.at(i)).y+whole_obstacles_point3.at(diff_point3_max_y.at(i)+1).y)/2.0))
         {
           Obstacle_index_through = diff_point3_max_y.at(i);
           mid_point_y = abs(robot_pose.y-(whole_obstacles_point3.at(diff_point3_max_y.at(i)).y+whole_obstacles_point3.at(diff_point3_max_y.at(i)+1).y)/2.0);
         }

    }
    return Obstacle_index_through;
}

int Function::get_stop_index(vector<Point3d> whole_obstacles_point3)
{
    float max_point_y = 255;
    int Obstacle_index_stop = Default_value;//255
    for(int i = 0;i<Stop_selecting_max.size();i++)
    {
      if(max_point_y>abs(robot_pose.y-(whole_obstacles_point3.at(Stop_selecting_max.at(i)).y+whole_obstacles_point3.at(Stop_selecting_max.at(i)+1).y)/2.0))
      {
        Obstacle_index_stop = Stop_selecting_max.at(i);
        max_point_y = abs(robot_pose.y-(whole_obstacles_point3.at(Stop_selecting_max.at(i)).y+whole_obstacles_point3.at(Stop_selecting_max.at(i)+1).y)/2.0);
      //  cout<<"max_point_y = "<<max_point_y<<"  Obstacle_index_stop = "<<Obstacle_index_stop<<endl;
      }
    }
    return Obstacle_index_stop;
}

float get_distance_fun(float point_x,float point_y)
{
  if(point_y<0) point_y = (-point_y);
  if(point_x<0) point_x = (-point_x);
  if(point_x>point_y){
   return  pow(point_y,0.5)+point_x-point_y;
  }
  else{
     return  pow(point_x,0.5)+point_y-point_x;
  }
}
float get_chance_value_of_relative_obstacle(float point_x,float point_y)
{
      float R = 0;
      float K = 0;
      if(point_x<0.6){
        R  = pow(point_y,2) + pow(point_x,2);
      }
      else{
          R  = pow(point_y,1) + pow(point_x,1);
      }
      //考虑 mid_point_y 变化过快 待验证
      // cout<<"  mid_y = "<<mid_y<<"  r_y = "<<robot_pose.y<<"  r_x = "<<robot_pose.x<<endl;
      if(R<0.5)  K = Current_speed*0.5;
      else
      {
          //K = Current_speed/(2*R+robot_L);//左右轮子速度和车体速度的差
            K = Current_speed/(2*R);//K = LV/（2R）
            if(K>Current_speed*0.5)
                K = Current_speed*0.5;
      }
     // cout<<"R_y = "<<robot_pose.y<<" mid_y = "<<mid_y<<endl;
      if(point_y>0)
      {
        K = -K;
        cout<<"向右"<<endl;
      }
      else{
        cout<<"向左"<<endl;
      }
   return 0;//测试/10 K/10
}
void robot_stop_fun()
{
     cout<<"                              相对坐标stop"<<endl;
     state = 2;
     oFile_init<<"robot_stop_funKKKKKKKKKKK1"<<endl;
}
void robot_run_fun()
{
     cout<<"                               相对坐标run"<<endl;
     oFile_init<<"robot_run_funKKKKKKKKKKK1"<<endl;
     state = 1;//0
    
}
float relative_obstacle_control(float point_x,float point_y)
{
  float value = 0;
  cout<<" robot_stop_fun point_x = "<<point_x<<" point_y = "<<point_y<<endl;
  if((point_x<(stop_rel_point_x))&&(abs(point_y)<stop_rel_point_y))
  {
     robot_stop_fun();
      value = 0;
  }
  else{
     robot_run_fun();
    value = get_chance_value_of_relative_obstacle(point_x,point_y);
  }
  return value;
}

float According_relative_obstacle_chance_speed()
{
 // Function obstacle_fun;
  float value = 0;
 // obstacle_fun.get_relative_control_point(_Relative_point3);
  cout<<"relative_obstacle_point3.x = "<<relative_obstacle_point3.x<<"  relative_obstacle_point3.y = "<<relative_obstacle_point3.y<<endl;
  value = relative_obstacle_control(relative_obstacle_point3.x,relative_obstacle_point3.y);
  return value;
}

/*
函数名：Avoidance_Callback
功能  ：订阅激光雷达数据 进行避障处理
说明  ：ranges：转一周的测量数据一共360个 可以理解为每个整数角度对应一个值 突出部分的值为ranges[180]

*/
void Avoidance_Callback(const sensor_msgs::LaserScan& LaserScan) 
{
 //  cout<<"Avoidance_Callback"<<endl;
   //for(int i = 160;i<200;i++)
   for(int i = 0;i<10;i++)
     Classification_group[i] = 0;
    
   for(int i = 90;i<=270;)
   {
      if(((i<=136)&&(i>=131))||((i<=227)&&(i>=220)))
      {

      }
      else
      {
            if(abs(i-180)<10)
            {
               if((LaserScan.ranges[i]<2.1)&&(state_one_flag == 0))//2.1
               {
                 state = 1;
                 flag = 1;
                // oFile_init<<"55555555KKKKKKKKKKK"<<endl;
                 cout<<"state = 1 进行减速"<<endl;
                // slow_down_laser[i] =  LaserScan.ranges[i];
               }
            }
            if(LaserScan.ranges[i]<1.8)//1.8
            {
              distan_Laser[i] = LaserScan.ranges[i];
              Classification_group[(i-90)/One_Area_Size]++;
            }
            else  distan_Laser[i] = 0;
      }
        i = i+lidar_gap;//5
   }     
 // for(int i = 0;i<10;i++)
//   cout<<i<<" "<<Classification_group[i]<<" ";
//   cout<<endl;
}

/*
函数名：get_current_speed_Callback
功能  ：订阅激AGV当前速度和amcl坐标
说明  ：
*/
void get_current_speed_Callback(const geometry_msgs::Twist& twist) 
{
  Current_speed = twist.linear.x;
  robot_pose.x = twist.angular.x;
  robot_pose.y = twist.angular.y;
  robot_pose.angle = twist.linear.z;//*57.29578
  cout<<"robot_pose.x = "<<robot_pose.x<<endl;
}






void return_amcl_get_pose1()
{
    tf::TransformListener listener_amcl_pose;
    tf::StampedTransform transform_amcl_pose;
    Eigen::Quaterniond q_amcl;
    ros::Time begin = ros::Time::now();
    listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
    listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
    q_amcl.x() = transform_amcl_pose.getRotation().getX();
    q_amcl.y() = transform_amcl_pose.getRotation().getY();
    q_amcl.z() = transform_amcl_pose.getRotation().getZ();
    q_amcl.w() = transform_amcl_pose.getRotation().getW();
    Eigen::Vector3d euler_linear = q_amcl.toRotationMatrix().eulerAngles(0, 1, 2);
    //cout << "  A = "<<euler_linear[2]*57.29578<<endl;
    ros::spinOnce();
    cout << "  amcl_W_x11 = "<<transform_amcl_pose.getOrigin().x() << "  W_y = "<<transform_amcl_pose.getOrigin().y()<< "  A = "<<euler_linear[2]*57.29578<<endl ;
}

 void GetMaxAndMin(float arr[])//
  {
   
		float max = arr[0];
    //效率比冒泡排序和选择排序高 时间复杂度为n*log2(n)
    sort(arr,arr+Divided_Amount);

	}//


/*
函数名：Avoidance_Callback1
功能  ：订阅激光雷达数据 进行简单的避障处理
说明  ：ranges：转一周的测量数据一共360个 可以理解为每个整数角度对应一个值 突出部分的值为ranges[180]

*/
void Avoidance_Callback1(const sensor_msgs::LaserScan& LaserScan) 
{
//   cout<<"Avoidance_Callback"<<endl;
   //for(int i = 160;i<200;i++)
   for(int i = 175;i<185;i++)
   {
        if(LaserScan.ranges[i]<1)
        {
          // cout<<"ranges = "<<LaserScan.ranges[i]<<endl;
          if(LaserScan.ranges[i]<0.5) 
          {
                state = 2;   
          }
          else 
          {
               state = 1;
          } 
         break;
        }
        else{
             state = 0;
        }
   }     
}

/*
函数名   ：Get_whole_obstacle_block_num
传入参数 ：whole_point3_vector *Obstacle_block_coordinates_whole &Obstacle_block_num_whole
功能    ：返回绝对障碍物块数 和障碍物边界的坐标。
说明    ：传入的为指针数组 和指针 函数内部参数值的改变将改变全局的参数值
*/
void Get_whole_obstacle_block_num(vector<Point3d> whole_point3_vector,int *Obstacle_block_coordinates_whole,int &Obstacle_block_num_whole)
{    

    for(int i = 0; i < whole_point3_vector.size(); i++)
     {
        float d_angle_w = 0;
        if(i<(whole_point3_vector.size()-1))
            d_angle_w=(whole_point3_vector.at(i+1).z-whole_point3_vector.at(i).z);
        if(abs(d_angle_w)>5)
           {
             Obstacle_block_coordinates_whole[Obstacle_block_num_whole] = i;
             Obstacle_block_num_whole++;
           }
     }
}

/*
函数名   Get_relative_obstacle_block_num
传入参数 point3 *Obstacle_block_coordinates &Obstacle_block_num
功能    ：返回相对障碍物块数 和障碍物边界的坐标。以车头方向为X 车头垂直方向为Y
说明    ：传入的为指针数组 和指针 函数内部参数值的改变将改变全局的参数值
*/
void Get_relative_obstacle_block_num(vector<Point3d> point3,int *Obstacle_block_coordinates,int &Obstacle_block_num)
{    
    for(int i = 0; i < point3.size(); i++)
      {
         float d_angle = 0;
          if(i>0)
            d_angle=(point3.at(i).z-point3.at(i-1).z);
          if(abs(d_angle)>5)
           {
             Obstacle_block_coordinates[Obstacle_block_num] = i-1 ;//
             Obstacle_block_num++;
           }
           
      }
}

int Filter_date(int date)
{
  int index_value  = date;
  int min_index    = 255;
  int flag_num     = 0;
  int index        = 0;
  save_date.push_back(date);
  if((save_date.size() == 1)||(save_date.size() == 2))
  {
    index_value = save_date.at(save_date.size()-1);
     return index_value;
  }
  else
  {
      for(int i = 0;i<save_date.size();i++)
       {
        if(save_date.at(i)==1024)
        {
          flag_num++;
        }
        cout<<"save_date.at(i) = "<<save_date.at(i)<<endl;
       }
      if(flag_num>1)  index_value = 1024;
      else
        {
           for(int i = 0;i<save_date.size();i++)
            {
                  if(min_index>save_date.at(i))
                  {
                    min_index = save_date.at(i);
                  }
            }
           index_value = min_index;
        }
        
      if(save_date.size()>2)
      {
        vector<int>::iterator k = save_date.begin();
        save_date.erase(k);//删除第一个元素
      }
      return index_value;
  }
}
/*
函数名   ：Deal_with_whole_pose
传入参数 ：whole_obstacles_point3 障碍物的全局坐标
功能    ：得到障碍物间的最大间隔值 并返回相应的障碍物的坐标值
说明    ：走廊两侧y轴坐标设置为(-0.9 +0.9)

*/
int test_num = 0;
void Deal_with_whole_pose(vector<Point3d> whole_obstacles_point3)
{
  float pre_y  = 0;
  float dy = 0;
  float final_chance_value = 0;
  Function obstacle_fun;
  boundary_point3.x = 255;
  boundary_point3.y = -Aisle_width_half;
  boundary_point3.z = 1023;
  whole_obstacles_point3.insert(whole_obstacles_point3.begin(),boundary_point3);
  boundary_point3.x = 255;
  boundary_point3.y = Aisle_width_half;
  boundary_point3.z = 1024;
  whole_obstacles_point3.push_back(boundary_point3);
  filter_mutation_z(whole_obstacles_point3);
  if(whole_obstacles_point3.size()>3)
    filter_mutation_y(whole_obstacles_point3);
  filter_max_y(whole_obstacles_point3);
 // cout<<" whole_obstacles_point3.size11() = "<< whole_obstacles_point3.size()<<endl;
 // get_max_obstacle_x = get_max_value_fun(obstacle_x);
  int point_size = whole_obstacles_point3.size();
  if(point_size>2) {
   // oFile_init<<"OOOOOOOOOOKKKKKKKKKKK"<<endl;
    state_one_flag = 1;
     state = 1;//0
  }
  oFile_init<<"whole_obstacles_point3KKKKKKKKKKKKKKKK"<<endl;
  obstacle_fun.print_vector(whole_obstacles_point3);
  float diff_point3_y[point_size-1]={0};
  Stop_selecting_max.clear();
  diff_point3_max_y.clear();
  obstacle_fun.get_diff_point_fun(whole_obstacles_point3,diff_point3_y);
  obstacle_fun.get_max_index(diff_point3_y);
 // oFile_init<<"size() = "<<whole_obstacles_point3.size()<<" x = "<<robot_pose.x<<" y = "<<robot_pose.y<<" robot_pose.angle = "<<robot_pose.angle<<endl;
  for(int i = 0;i<point_size-1;i++)
   {
    //  cout<<"diff_point3_y[i] = "<<diff_point3_y[i]<<endl;
      if(diff_point3_y[i]>=0.7)
      {
        diff_point3_max_y.push_back(i);
      }
      else
      {
        Stop_selecting_max.push_back(i);
      }
   }
    Obstacle_index_through = obstacle_fun.get_through_index(whole_obstacles_point3);
   // Obstacle_index_through = Filter_date(Obstacle_index_through);
    cout<<"obstacle_fun .Obstacle_index= "<<Obstacle_index_through<<endl;
    if(Obstacle_index_through == Default_value)
    {
        final_chance_value = According_relative_obstacle_chance_speed();
         cout<<"靠近障碍物 = "<<final_chance_value<<endl;
    }
   // if(Obstacle_index_through == Default_value)
   //  Obstacle_index_stop = obstacle_fun.get_stop_index(whole_obstacles_point3);
  // cout<<"Obstacle_index_stop = "<<Obstacle_index_stop<<"  Obstacle_index_through = "<<Obstacle_index_through<<endl;
    else{
        final_chance_value = According_to_obstacles_chance_speed(whole_obstacles_point3,Obstacle_index_through,Obstacle_index_stop);
        cout<<"通过障碍物 = "<<final_chance_value<<endl;
    }
    Speed_change = final_chance_value/Robot_width_big;
    cout<<"Speed_change = "<<Speed_change<<" Current_speed = "<<Current_speed<<endl;
   // Speed_change = 0.2;
}


/*
函数名   ：Deal_with_whole_pose
传入参数 ：whole_obstacles_point3 障碍物的全局坐标
功能    ：得到障碍物间的最大间隔值 并返回相应的障碍物的坐标值
说明    ：走廊两侧y轴坐标设置为(-0.9 +0.9)

*/
void Select_route(vector<Point3d> whole_obstacles_point3)
{
    for(int i = 0;i<diff_point3_max_y.size();i++)
      {
           cout<<"g = "<<whole_obstacles_point3.at(diff_point3_max_y.at(i)).y<<endl;
           //cout<<"g = "<<whole_point3_vector.at(diff_point3_max_y.at(i+1)).y<<endl;
      }

}
/*
函数名   According_to_obstacles_chance_speed1
传入参数 ：
功能    ：根据得到的障碍物信息进行 修改速度进行机器人的避障
说明    ：走廊两侧y轴坐标设置为(-0.9 +0.9)

*/
void According_to_obstacles_chance_speed1(vector<Point3d> obstacles_point3,int Obstacle_block_array[],vector<Point3d> whole_obstacles_point3,int whole_Obstacle_block_array[])
{

      int   Obstacle_block_array_num       = 0;
      int   whole_Obstacle_block_array_num = 0;
      float point_x = 0;
      float point_y = 0;
      float point_whole_x = 0;
      float point_whole_y = 0;
      int   obstacles_num = 255;
      int   obstacles_whole_num = 255;
      float R = 0;
      float K = 0;
      float min_whole_Obstacl_y = 0;
      obstacles_num = obstacles_point3.size();
      obstacles_whole_num = obstacles_point3.size();
     // cout<<" According_topoints_y.size() = "<< obstacles_point3.size()<<endl;
      if(obstacles_num>0)
      {
        //相对坐标
         for(int i = 0;Obstacle_block_array[i]!='\0';i++)
         {
 //          if(obstacles_num>0)
              cout<<" y = "<<obstacles_point3.at(Obstacle_block_array[i]).y<<endl;
              cout<<"i = "<<i<<" obstacles_num = "<<obstacles_num<<endl;//i=0 一个障碍物
              Obstacle_block_array_num = i;
         }

        //全局坐标
        for(int i = 0;whole_Obstacle_block_array[i]!='\0';i++)
         {
 //          if(obstacles_num>0)
              for(int j = 0;j<whole_Obstacle_block_array[j];j++)
             // for(int i = whole_Obstacle_block_array_num;i<whole_Obstacle_block_array[i];i++)
              {
                min_whole_Obstacl_y = whole_obstacles_point3.at(0).y;
                if(min_whole_Obstacl_y>whole_obstacles_point3.at(whole_Obstacle_block_array[i]).y)
                  min_whole_Obstacl_y = whole_obstacles_point3.at(whole_Obstacle_block_array[i]).y;
              }
              cout<<" 全局y = "<<whole_obstacles_point3.at(whole_Obstacle_block_array[i]).y<<endl;
              cout<<"i = "<<i<<" whole_size = "<<whole_obstacles_point3.size()<<endl;//i=0 一个障碍物
              whole_Obstacle_block_array_num = i;
         }
         
         switch(Obstacle_block_array_num)
         {
            case 0:
                cout << "存在1个障碍物" <<endl;
                point_x = obstacles_point3.at(int(obstacles_num/2)).x;
                point_y = obstacles_point3.at(int(obstacles_num/2)).y;
                point_whole_x = whole_obstacles_point3.at(int(obstacles_whole_num/2)).x;
                point_whole_y = whole_obstacles_point3.at(int(obstacles_whole_num/2)).y;
                //R = (abs(point_y) + point_x - 0.1);
                R  = pow(point_y,2)*1.2 + pow(point_x,2);
                cout<<"point_x ="<<point_x<<" point_y = "<<point_y<<" R = "<<R<<" point_whole_y = "<<point_whole_y<<endl;
                Current_speed = 0.2;
                if(R<0.5)
                 K = Current_speed*0.7;
                else
                {
                //K = Current_speed/(2*R+robot_L);//左右轮子速度和车体速度的差
                  K = Current_speed/(2*R);//K = LV/（2R）
                  if(K>Current_speed*0.6)
                     K = Current_speed*0.6;
                }
                if(point_y<=0)
                K = -K;
                cout<<"K = "<<K<<" R = "<<R<<" Current_speed = "<<Current_speed<<endl;
                state = 0; 
              break;
            case 1:
             // state = 2; //停止
             // K = 0;
              //cout<<" point0 ="<<Obstacle_block_array[0]<<" point1 ="<<Obstacle_block_array[1]<<endl;
              cout << "存在2个障碍物" <<"obstacle_y = "<<obstacle_y<<endl;
              break;
            case 2:
             // state = 2; //停止
             // K = 0;
              cout << "存在3个障碍物" <<endl;
              break;
            default:
            cout << "存在多个障碍物" <<endl;
            //  K = 0;
            //  state = 2;
        }
      }
      else
      {
        K = 0;
        state = 0;
        cout << "不存在障碍物" << endl;
      }
      cout<<" R  = "<< R <<" K = "<<K<<endl;
      Speed_change = K;
}

float Robot_Rotate_Avoidance()
{
   // Function obstacle_fun;
    float value = 0;
  //  obstacle_fun.get_relative_control_point(_Relative_point3);
    cout<<"relative_obstacle_point3.x = "<<relative_obstacle_point3.x<<"  relative_obstacle_point3.y = "<<relative_obstacle_point3.y<<endl;
    float point_x = relative_obstacle_point3.x;
    float point_y = relative_obstacle_point3.y;
    return abs(point_y);
    //return abs(point_y)+point_x;
}

float get_Arc_control_vaule(float E,float angle)
{
  float vaule = 0;
  cout<<"E = "<<E<<" angle = "<<angle<<"  Current_speed = "<<Current_speed<<endl;
  vaule = Current_speed*sin(angle/2)*sin(angle/2)/E;
  return vaule;
}
float get_chance_value(float obstacles_x_one,float obstacles_y_one,float mid_y)
 {
      float R = 0;
      float K = 0;
      float dx_one = abs(obstacles_x_one-robot_pose.x);
      float dy_one = abs(obstacles_y_one-robot_pose.y);
      float distance = sqrt(pow(dy_one,2) + pow(dx_one,2));
      cout<<"distance = "<<distance<<" dx_one = "<<dx_one<<" dy_one = "<<dy_one<<endl; 
      if(dy_one<0.5)
      {
              if(dx_one<0.6){
                 // R  = pow(dy_one,3) + pow(dx_one,3);//2
                  R  = 1024;//1.7 不调节 R 取很大的值
                  oFile_init<<"dx_one R = "<<R<<endl;
                  cout<<"dx_one<0.6 R = "<<R<<endl;
              }
              else{
                      if(dx_one>1.0)
                      {
                        R  = (pow(dy_one,5) + pow(dx_one,5));//
                      //  if(dx_one>1.2){
                          //if(dy_one<0.29)
                            if(R<1.7) R = 1.7;
                            // R = 1;
                      //   }
                      }
                    else{
                      R  = 1.5;// 1.5 pow(dy_one,0.5) + pow(dx_one,0.5)
                    }
                  //  oFile_init<<"else R = "<<R<<endl;
                      cout<<"else  R = "<<R<<endl;
                    //  if(R<1.7) R = 1.7;
              }
              
              //考虑 mid_point_y 变化过快 待验证
              // cout<<"  mid_y = "<<mid_y<<"  r_y = "<<robot_pose.y<<"  r_x = "<<robot_pose.x<<endl;
              //Current_speed = 0.2;
              if(R<0.5)  K = Current_speed*0.5;
              else
              {
                  //K = Current_speed/(2*R+robot_L);//左右轮子速度和车体速度的差
                    K = Current_speed/(2*R);//K = LV/（2R）
                    if(K>Current_speed*0.5)
                        K = Current_speed*0.5;
                    if(R == 1024)
                     K = 0;
              }
     }
     else{
         K = 0;
     }
//      oFile_init<< "dx_one"<< " "<<dx_one<<"  "<< "dy_one"<< " "<<dy_one<<"  "<<" x = "<<robot_pose.x<<"  "<<" R = "<<R<<"  "<<" K = "<<K<<endl;
      float K1 = get_Arc_control_vaule(abs(mid_y-robot_pose.y),abs(robot_pose.angle)+P_i/4);
      
     // cout<<"R_y = "<<robot_pose.y<<" mid_y = "<<mid_y<<endl;
      cout<<"R = "<<R<<"   K = "<<K<<"   K1 = "<<K1<<endl;
    //  K = K1;
      float _point_y = Robot_Rotate_Avoidance();
      // cout<<"原地旋转 _point_y = "<<_point_y<<endl;
      float point_x = relative_obstacle_point3.x;
      float point_y = relative_obstacle_point3.y;
      //if(((distance<0.6)&&(_point_y<0.35))||(flag == 1)||((point_x<stop_rel_point_x)&&(abs(point_y)<stop_rel_point_y)))
      cout<<" 测试point_x = "<<point_x<<" point_y = "<<point_y<<"  flag = "<<Continued<<endl;
      if(((Continued == 1)&&(point_x<(stop_rel_point_x+0.2)))||((point_x<stop_rel_point_x+0.15)&&(abs(point_y)<(stop_rel_point_y-0.05))))
      //if(((Continued == 1)&&(point_x<(stop_rel_point_x+0.2))&&(abs(point_y)<(stop_rel_point_y+0.05)))||((point_x<stop_rel_point_x+0.15)&&(abs(point_y)<(stop_rel_point_y-0.05))))
      {
        state = 2;
        Continued  = 1;
        oFile_init<<"222222222222222KKKKKKKKKKK"<<" point_x "<<point_x<<" point_y  "<<point_y<<" Continued  "<<Continued<<endl;
        cout<<"distance = "<<distance<<"_point_y = "<<_point_y<<endl;
        cout<<"原地旋转                      全局接近目标点stop"<<endl;
        if(abs(robot_pose.angle*57.29578)<80)
          K = ratation_value;//0.2  Current_speed
        else K = 0;
      }
      else{
        cout<<"                              全局接近目标点正常运行"<<endl;
      }
      cout<<"Current_speed = "<<Current_speed<<endl;
      //  K = 0.2;
      if((robot_pose.y-mid_y)>0)
      {
        K = -K;
//        oFile_init<< "To the right"<< " "<<" x = "<<robot_pose.x<<endl;
        cout<<"向右"<<endl;
      }
      else{
//        oFile_init<< "To the left"<< " "<<" x = "<<robot_pose.x<<endl;
        cout<<"向左"<<endl;
      }
   return K;
 }
/*
实例：              
closest_x = get_closest_x(get_Obstacle_index,obstacles_x_one,obstacles_y_one,obstacles_x_two,obstacles_y_two,obstacles_num);
cout<<"closest_x = "<<closest_x<<endl;
*/
float get_closest_x(int get_Obstacle_index,float obstacles_x_one,float obstacles_y_one,float obstacles_x_two,float obstacles_y_two,int obstacles_num)
{
    float closest_x         = 0;
    if((get_Obstacle_index == 0))
      closest_x = obstacles_x_two;
    else if(get_Obstacle_index == (obstacles_num-1))//(obstacles_point3.size()-1)
      {
        closest_x = obstacles_x_one;
      }
    else
    {
      if(abs(obstacles_y_one-robot_pose.y)>abs(obstacles_y_two-robot_pose.y))
        closest_x = obstacles_x_two;
      else  closest_x = obstacles_x_one;
    }
  return closest_x;
}


/*
函数名   special_position_control
传入参数 ： y 为全局y坐标 A 为全局机器人的角度
功能    ：当机器人靠近中心线 但是车头方向偏离很多时 进行矫正
说明    ：

*/
float special_position_control(float x,float y,float A)
{
   float K_value = 0;
   A = A*57.29578;
   //if((abs(y)<0.1)&&(abs(A)>25))
   cout<<" y = "<<y<<" A = "<<A<<endl;
   if((abs(y)<0.2)&&(abs(A)>10))
   {
     // state = 2;
      if(A<0)
      K_value = ratation_value;//*1.5
      else
      K_value = (-ratation_value);//*1.5
      cout<<"已远离障碍物 进入旋转体调节"<<endl;
   }
   else K_value = 0;
   return K_value;
}


float Robot_Boundary_deal(float x,float y,float A)
{
  float K_value = 0;
  
  if(y>Robot_Boundary)
  {
    if(A>0)
    {
      cout<<"进入边界区域 进入旋转体调节 1"<<endl;
      K_value = (-ratation_value*0.5);
      state = 2;
      oFile_init<<"333333333333KKKKKKKKKKK"<<endl;

    }
  }
  if(y<(-Robot_Boundary))
  {
    if(A<0)
    {
       cout<<"进入边界区域 进入旋转体调节 2"<<endl;
      K_value = ratation_value*0.5;
      state = 2;
      oFile_init<<"444444444444KKKKKKKKKKK"<<endl;

    }
  }

  return K_value;
}

/*
函数名   According_to_obstacles_chance_speed
传入参数 ：
功能    ：根据得到的障碍物信息进行 修改速度进行机器人的避障
说明    ：走廊两侧y轴坐标设置为(-0.9 +0.9)

*/
float According_to_obstacles_chance_speed(vector<Point3d> obstacles_point3,int pass_index,int stop_index)
{
      Function obstacle_fun;
      float point_x = 0;
      float point_y = 0;
      int   obstacles_num = 255;
      float R = 0;
      float K = 0;
      float adjust_amcl_Y = 0;
      float mid_y   = 0;
      float obstacles_x_one   = 0;
      float obstacles_x_two   = 0;
      float obstacles_y_one   = 0;
      float obstacles_y_two   = 0;
      int obstacles_lidar_z = 0;
      int _obstacles_lidar_z = 0;
      float closest_x         = 0;
      int   get_Obstacle_index  = 0;
      get_Obstacle_index = pass_index;
      if(get_Obstacle_index>1000) cout<<"get_Obstacle_index 越界"<<endl;
//      cout<<"get_Obstacle_index = "<<get_Obstacle_index<<endl;
      obstacles_x_one = obstacles_point3.at(get_Obstacle_index).x;
      obstacles_y_one = obstacles_point3.at(get_Obstacle_index).y;
      obstacles_x_two = obstacles_point3.at(get_Obstacle_index+1).x;
      obstacles_y_two = obstacles_point3.at(get_Obstacle_index+1).y;
//      cout<<"obstacles_x_one = "<<obstacles_x_one<<"obstacles_y_one = "<<obstacles_y_one<<endl;
//      cout<<"obstacles_x_two = "<<obstacles_x_two<<"obstacles_y_two = "<<obstacles_y_two<<endl;
      mid_y = (obstacles_y_one+obstacles_y_two)/2;
      if(get_Obstacle_index == 0)
      { 
        obstacles_x_one = obstacles_x_two;
        obstacles_y_one = obstacles_y_two;
        obstacles_lidar_z = obstacles_point3.at(get_Obstacle_index+1).z;
      }
      else{
         obstacles_lidar_z = obstacles_point3.at(get_Obstacle_index).z;
         if(obstacles_point3.at(get_Obstacle_index+1).z!=1024)
         {
           obstacles_lidar_z = (obstacles_point3.at(get_Obstacle_index).z+obstacles_point3.at(get_Obstacle_index+1).z)/2;
         }

      } 
      obstacles_num = obstacles_point3.size();
      state = 1;
      if(flag == 0)
      {
         state = 0; 
         oFile_init<<"flag0flag0KKKKKKKKKKKKKKK"<<endl;

      }
      cout<<" relative_obstacle_point3.y = "<<relative_obstacle_point3.y<<" obstacles_lidar_z = "<<obstacles_lidar_z<<" obstacles_num = "<<obstacles_num<<" obstacles_y_one"<<obstacles_y_one<<endl;
     // oFile_init<<"obstacles_y_one="<<obstacles_y_one<<" obstacles_x_one = "<<obstacles_x_one<<" re_ob_point = "<<relative_obstacle_point3.y<<endl;
      oFile_init<<"obstacles_lidar_zKKKKKKKKKKKKKKK = "<< obstacles_lidar_z<<"d_L[z] "<<distan_Laser[obstacles_lidar_z]<<endl;
      if(abs(180-obstacles_lidar_z)>=48)
      {
        Continued = 0;
      }
       
      if((obstacles_num>2)&&((abs(180-obstacles_lidar_z)<=48)||(distan_Laser[obstacles_lidar_z]<0.5))&&(abs(relative_obstacle_point3.y)<0.55))
      {
        oFile_init<<"CCCCCCCCCCCC0KKKKKKKKKKKKKKK"<<endl;
        cout<<"存在障碍物"<<endl;
      //  oFile_init<<"exist obstacles exist distan_Laser[obstacles_lidar_z] = "<<distan_Laser[obstacles_lidar_z]<<endl;
       // cout<<"robot_pose.x  = "<<robot_pose.x <<" obstacles_lidar_z = "<<obstacles_lidar_z<<endl;
        K = get_chance_value(obstacles_x_one,obstacles_y_one,mid_y);
      //  cout<<"K = "<<K<<endl;
      //  cout<<"get_max_obstacle_x = "<<get_max_obstacle_x<<endl;
      }
      else
      {
        oFile_init<<"BBBBBBBBBBBBBBCCCKKKKKKKKKKKKKKK"<<endl;
      //  oFile_init<<"no obstacles no obstacles no obstacles no obstacles="<<endl;
        cout<<"                                   不存在障碍物"<<endl;
        Continued = 0;
       // flag = 0;
        K = 0;
        cout<<"get_max_obstacle_x = "<<get_max_obstacle_x<<"  robot_pose.x = "<<robot_pose.x<<endl;
        if(abs(robot_pose.y)<=Robot_Boundary)
        {
           cout<<"rx-gx =  "<<robot_pose.x-get_max_obstacle_x<<endl;
            //if(abs(robot_pose.x-get_max_obstacle_x)>1.0)//1.0

            if((robot_pose.x-get_max_obstacle_x)>0.3)//1.0
            {
              K = special_position_control(robot_pose.x,robot_pose.y,robot_pose.angle);
              if(get_max_obstacle_x>0)
              {
              //   flag = 0;
                state_one_flag = 0;
              }
             // oFile_init<<"state_one_flag "<<state_one_flag<<" r.x "<<robot_pose.x<<" get_max_obstacle_x "<<get_max_obstacle_x<<endl;
            }
        }
        else
        {
            K = Robot_Boundary_deal(robot_pose.x,robot_pose.y,robot_pose.angle);
        }   
      }
    cout<<"flag = "<<flag<<endl;
    return K;
}

/*
函数名   According_to_obstacles_chance_speed
传入参数 ：
功能    ：根据得到的障碍物信息进行 修改速度进行机器人的避障
说明    ：走廊两侧y轴坐标设置为(-0.9 +0.9)
Relative coordinates
*/
void According_to_Relative_obstacles_chance_speed(vector<Point3d> obstacles_point3,int pass_index,int stop_index)
{
      float point_x = 0;
      float point_y = 0;
      int   obstacles_num = 255;
      float R = 0;
      float K = 0;
      float adjust_amcl_Y = 0;
      float mid_y   = 0;
      float obstacles_x_one   = 0;
      float obstacles_x_two   = 0;
      float obstacles_y_one   = 0;
      float obstacles_y_two   = 0;
      float obstacles_lidar_z = 0;
      float closest_x         = 0;
      int get_Obstacle_index  = 0;
      if(pass_index == Default_value)   
      {
      //  cout<<"靠近障碍物 不通过"<<endl;
        get_Obstacle_index = stop_index;
      } 
      else
      {
       //  cout<<"通过障碍物"<<endl;
         get_Obstacle_index = pass_index;
      }
      if(get_Obstacle_index>1000) cout<<"get_Obstacle_index 越界"<<endl;
      cout<<"get_Obstacle_index = "<<get_Obstacle_index<<endl;
      obstacles_x_one = obstacles_point3.at(get_Obstacle_index).x;
      obstacles_y_one = obstacles_point3.at(get_Obstacle_index).y;
      obstacles_x_two = obstacles_point3.at(get_Obstacle_index+1).x;
      obstacles_y_two = obstacles_point3.at(get_Obstacle_index+1).y;
      mid_y = (obstacles_y_one+obstacles_y_two)/2;
      if(get_Obstacle_index == 0)
      { 
        obstacles_x_one = obstacles_x_two;
        obstacles_y_one = obstacles_y_two;
        obstacles_lidar_z = obstacles_point3.at(get_Obstacle_index+1).z;
      }
      else{
         obstacles_lidar_z = obstacles_point3.at(get_Obstacle_index).z;
      }
      obstacles_num = obstacles_point3.size();
     // cout<<" obstacles_lidar_z = "<< obstacles_lidar_z<<endl;
      state = 0; 
      if(pass_index == Default_value) 
      {
        //if((abs(obstacles_y_one-robot_pose.y)<0.5)||(abs(obstacles_x_one-robot_pose.x)<0.5))
    //    cout<<"O_X = "<<obstacles_x_one<<"  R_X = "<<robot_pose.x<<" O_Y = "<<obstacles_y_one<<endl;
        if(abs(obstacles_x_one-robot_pose.x)<0.55&&(abs(obstacles_y_one-robot_pose.y)<0.6))
        {
          state = 2;
          K = 0; 
          cout<<"停止 state = "<<state<<"  robot_pose.x = "<<robot_pose.x<<"  robot_pose.y = "<<robot_pose.y<<endl;
        }
        
      }
      if(state == 0)
      {
            if(obstacles_num>2&&(abs(180-obstacles_lidar_z)<=35))
            {
        //          cout<<"存在障碍物"<<endl;
                 // cout<<"z = "<<obstacles_point3.at(get_Obstacle_index).z<<endl;
                  if((get_Obstacle_index == 0))
                  closest_x = obstacles_x_two;
                  else if(get_Obstacle_index == (obstacles_point3.size()-1))
                    {
                      closest_x = obstacles_x_one;
                    }
                  else
                  {
                    if(abs(obstacles_y_one-robot_pose.y)>abs(obstacles_y_two-robot_pose.y))
                      closest_x = obstacles_x_two;
                    else  closest_x = obstacles_x_one;
                  }
                  float dx_one = abs(obstacles_x_one-robot_pose.x);
                  float dy_one = abs(obstacles_y_one-robot_pose.y);
                 // cout<<"dy = "<<obstacles_y_one-robot_pose.y<<"dx = "<<obstacles_x_one-robot_pose.x<<endl;
                  //if(obstacles_x_one<0.6)
                  if(dx_one<0.6){
                    R  = pow(dy_one,2) + pow(dx_one,2);
                  //  cout<<"pow2 = R = "<<R<<endl;
                  }
                  else{
                     R  = pow(dy_one,1) + pow(dx_one,1);
                  }
                  //考虑 mid_point_y 变化过快 待验证
                 // cout<<"  mid_y = "<<mid_y<<"  r_y = "<<robot_pose.y<<"  r_x = "<<robot_pose.x<<endl;
                  if(R<0.5)  K = Current_speed*0.5;
                  else
                  {
                      //K = Current_speed/(2*R+robot_L);//左右轮子速度和车体速度的差
                        K = Current_speed/(2*R);//K = LV/（2R）
                        if(K>Current_speed*0.5)
                            K = Current_speed*0.5;
                  }
                  cout<<"R_y = "<<robot_pose.y<<" mid_y = "<<mid_y<<endl;
                  if((robot_pose.y-mid_y)>=0)
                  {
                    K = -K;
                    cout<<"向右"<<endl;
                  }
                  else{
                    cout<<"向左"<<endl;
                  }
                //  cout<<"K = "<<K<<" R = "<<R<<" Current_speed = "<<Current_speed<<endl;
            }
            else
            {
              K = 0;
            //  state = 0;
        //      cout << "不存在障碍物" << endl;
            }
      }
     
    //  cout<<" R  = "<< R <<" K = "<<K<<endl;
      Speed_change = K;
}
void chance_action(int mode)
{
      switch(mode)
        {
          case 0:
            Speed_change = 0.096;
            cout <<  "障碍物在右下" << endl;
            break;
          case 1:
            Speed_change = 0.086;
            cout << "右上" << endl;
            break;
          case 2:
            Speed_change = -0.086;
            cout << "左上" << endl;
            break;
          case 3:
            Speed_change = -0.096;
            cout << "左下" << endl;
            break;
          default:
            Speed_change = 0;
            cout << "不存在障碍物" << endl;
       }
}


/*
函数名  filter_obstacle_y
功能：求两个数的平均值
说明：仅适合一堆正 然后一堆负的 数值进行滤波不能交叉 
     正确：1 2 3 -3 -2
     不正确：1 2 3 -3 -2 2
设置滤波函数的目的 
 解决：红色标注部分为车体前无障碍物的情况 红色箭头标注的部分可以理解为异常数据的情况
在检测边缘障碍物坐标时 大部分障碍的坐标均大于1.0   出现个别障碍物的横坐标小于1.0
*/

float filter_obstacle_y(float f_obstacle_y)
{
   float value = 0;
 //  cout<<"f_obstacle_y = "<<f_obstacle_y<<endl;
   if(f_obstacle_y>=0)
   {
            if(obstacle_y_positive_num == 0)
            {
                value = 110;//舍弃f_obstacle_y*10
            }
            else{
                value = (f_obstacle_y+pre_f_obstacle_y)/2.0;
            }
            obstacle_y_positive_num++;
   }
   else{
            if(obstacle_y_negative_num == 0){
                value = -110;//舍弃f_obstacle_y*10
            }
            else{
                value = (f_obstacle_y+pre_f_obstacle_y)/2.0;
            }
            obstacle_y_negative_num++;
   }
   pre_f_obstacle_y = f_obstacle_y;
   return value;
}

float get_min_value_fun(float Settings)
{
  if(get_min_value>Settings)
   get_min_value = Settings;
 // cout<<"Settings = "<<Settings<<"    get_max_value = "<<get_max_value<<endl;
  return get_min_value;
}
float get_max_value_fun(float Settings)
{
  if(get_max_value<Settings)
   get_max_value = Settings;
 // cout<<"Settings = "<<Settings<<"    get_max_value = "<<get_max_value<<endl;
  return get_max_value ;
}

/*
函数名  filter_mutation_i
功能：过滤非连续出现的雷达角度 “i”
说明：考虑了第一个数据 和 最后一个数据的情况
*/
void filter_mutation_i(vector<Point3d> &whole_obstacles_point3)
{
    Function obstacle_fun;
    vector<Point3d> _whole_obstacles_point3;
    if(whole_obstacles_point3.size()>1)
    {
          for(int i = 0; i < (whole_obstacles_point3.size());i++)
          {
                if( i == 0)
                {
                    float cur_z  = whole_obstacles_point3.at(i).z;
                    float fut_z  = whole_obstacles_point3.at(i+1).z;
                    if(abs(cur_z - fut_z)<Error_control_vaule)
                      {
                        _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
                      }
                }
                else if(i == (whole_obstacles_point3.size()-1))
                {
                      float pre_z  = whole_obstacles_point3.at(i-1).z;
                      float cur_z  = whole_obstacles_point3.at(i).z;
                      if(abs(cur_z - pre_z)<Error_control_vaule)
                      {
                        _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
                      }
                }
                else
                {
                      float pre_z  = whole_obstacles_point3.at(i-1).z;
                      float cur_z  = whole_obstacles_point3.at(i).z;
                      float fut_z  = whole_obstacles_point3.at(i+1).z;
                      if((abs(cur_z - pre_z)<Error_control_vaule)||(abs(cur_z - fut_z)<Error_control_vaule))
                      {
                        _whole_obstacles_point3.push_back(whole_obstacles_point3.at(i));
                      }  
                } 
           }
    }
   //   cout<<"             实验1"<<endl;
   // obstacle_fun.print_vector(_whole_obstacles_point3);
   whole_obstacles_point3.assign(_whole_obstacles_point3.begin(), _whole_obstacles_point3.end());
  // obstacle_fun.print_vector(_whole_obstacles_point3);
}


/*
函数名：get_obstacle_point_fun
功能  ：保存障碍物坐标 并做出相应的处理
说明  ：
*/
void get_obstacle_point_fun()
{
    int Obstacle_block_coordinates[10]={0};
    int Obstacle_block_num = 0;//
    int Obstacle_block_coordinates_whole[10]={0};
    int Obstacle_block_num_whole = 0;
    std::vector<float> get_distance_angle(distan_Laser,distan_Laser+360);
    vector<Point2d> points;
    vector<Point3d> whole_point3_vector;
    std::vector<float>  points_y(distan_Laser,distan_Laser);
    float x = 0;
    float y = 0;
    float pre_date = 0;
    int test_pp  = 0;
    _Relative_point3.clear();
    Function obstacle_fun;
    pre_f_obstacle_y = 0;
    obstacle_y_positive_num = 0;
    obstacle_y_negative_num = 0; 
    get_min_value = Default_value;
//    get_max_obstacle_x      = 0;
    get_max_value           = 0;
    //obstacle_x = 0;
    //obstacle_y = 0;
  //  float min_y = 10;
    ros::Time begin = ros::Time::now();
    cout<<" robot_pose.x = "<< robot_pose.x<<" robot_pose.y = "<< robot_pose.y<<endl;
    int flag_max_dis = 0;
    for(int i = 90;i<=270;)//ros::spinOnce();加入此代码可以进入调阅函数
      {
        float max_dis = get_distance_angle[i]*sin(P_i/2.0-robot_pose.angle+(((180-i)/180.0)*P_i));
        if(((get_distance_angle[i] > _min_lidar_date)&&(max_dis<1.5))||((flag_max_dis==1)&&(max_dis<1.6)&&(get_distance_angle[i] > _min_lidar_date)))
         {
           // state = 1;
            if(max_dis>1.4) flag_max_dis = 1;
            obstacle_x = robot_pose.x+get_distance_angle[i]*sin(P_i/2.0-robot_pose.angle+(((180-i)/180.0)*P_i));
            obstacle_y = robot_pose.y+get_distance_angle[i]*cos(P_i/2.0-robot_pose.angle+(((180-i)/180.0)*P_i));
            float _obstacle_y=filter_obstacle_y(obstacle_y);
            test_pp++;
            if((abs(_obstacle_y)<Aisle_width_half)&&(abs(obstacle_y)<Aisle_width_half))//1.0
            {
            //   cout<<" i = "<<i<<" get_distance_angle[i] = "<<get_distance_angle[i]<<" robot_pose.angle = "<<robot_pose.angle<<" obstacle_x = "<<obstacle_x<<" obstacle_y = "<<obstacle_y<<endl;
               
               whole_point3.x = obstacle_x ;//- robot_pose.x
               whole_point3.y = obstacle_y ;//- robot_pose.y
            //   cout<<"obstacle_x = "<<obstacle_x<<"  get_max_value = "<<get_max_value<<"  get_max_obstacle_x = "<<get_max_obstacle_x<<endl;
             //  oFile_init<<"obstacle_x "<<obstacle_x<<" "<<"obstacle_y "<<obstacle_y<<" "<<" rx = "<<robot_pose.x<<" ry = "<<robot_pose.y<<" get_d_a = "<<get_distance_angle[i]<<" i = "<<i<<" robot_pose.angle = "<<robot_pose.angle<<endl;
              // get_max_obstacle_x = get_max_value_fun(obstacle_x);
               whole_point3.z = i;//robot_pose.angle
               whole_point3_vector.push_back(whole_point3);
               obstacle_fun.get_relative_obstacle_point3(get_distance_angle,i);
            }
            
          }
          i = i+lidar_gap;//5
      }
      cout<<"state = "<<state<<endl;
     // oFile_init<< "state = "<<state<<" x = "<<robot_pose.x<<" y = "<<robot_pose.y<<endl;
    //  cout<<"get_max_obstacle_x = "<<get_max_obstacle_x<<endl;
    //Result_pose min_pose = obstacle_fun.get_min_x_y(_Relative_point3);
   // cout<<"min_pose min_x = "<<min_pose.x<<"get_min_x_y min_y = "<<min_pose.y<<endl;
    filter_mutation_i(_Relative_point3);
    oFile_init<<"_Relative_point3KKKKKKKKKKKKKKKKKKKKKK"<<endl;
    obstacle_fun.print_vector(_Relative_point3);
    obstacle_fun.get_relative_control_point(_Relative_point3);
    
   // obstacle_fun.print_vector(_Relative_point3);
 //   cout<<" obstacle_fun y = "<< obstacle_fun.get_relative_control_point(_Relative_point3)<<endl;
    Get_relative_obstacle_block_num(_Relative_point3,Obstacle_block_coordinates,Obstacle_block_num);
    Obstacle_block_coordinates[Obstacle_block_num] = _Relative_point3.size()-1; 
  //  cout<<"Obstacle_block_num = "<<Obstacle_block_num<<endl;

    Deal_with_whole_pose(whole_point3_vector);
}
int main(int argc, char **argv)
{ 
    ros::init(argc,argv,"Processing_lidar_data");
    ros::NodeHandle n;
    ros::Publisher send_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",2);//改为1
    ros::Publisher custom_msg_pub = n.advertise<custom_msg_topic::custom_msg>("Avoidance_info", 2);
    ros::Subscriber get_scan = n.subscribe("scan", 1000, Avoidance_Callback);
    ros::Subscriber get_current_speed = n.subscribe("send_pose", 10, get_current_speed_Callback);
    //ros::Subscriber get_current_speed = n.subscribe("cmd_vel_pose", 10, get_current_speed_Callback);
    geometry_msgs::Twist twist;
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    //send_vel.publish(twist);
    ros::Rate loop_rate(10);
    int Max_index = 0;
    oFile_init.open("Processing_lidar_data_V1.csv",ios::out|ios::trunc);
    tf::StampedTransform transform; 
    custom_msg_topic::custom_msg msg;
    //float array[3] = {1.1,1.2,0.3};
    std::vector<float> Obstacle_information(Classification_group,Classification_group+10);
    msg.Avoidance_Classification_group = Obstacle_information;
   
   while(ros::ok())
    {
        ros::spinOnce();
        std::vector<float> Obstacle_information(Classification_group,Classification_group+Divided_Amount);
        msg.Avoidance_Classification_group = Obstacle_information;
        msg.levels_of_anger = state;
        //msg.min_lidar_data  = 2.1;
        vector<float>::iterator max = max_element(Obstacle_information.begin(), Obstacle_information.end());
       // cout<< "max = "<<*max <<" at position =" << distance(Obstacle_information.begin(), max) << endl;
        if(*max>5)
        {
          Max_index = distance(Obstacle_information.begin(), max);
        }
        else Max_index = No_Obstacles;//

      //  amcl_get_pose();
     // return_amcl_get_pose(); 
     // chance_action(Max_index);
     // _Relative_point3.clear(); 待测试
      cout<<"  robot_pose.x = "<<robot_pose.x<<"  robot_pose.y = "<<robot_pose.y<<endl;
    //  oFile_init<<"state = "<<state<<" get_max_obstacle_x "<<get_max_obstacle_x<<endl;
      get_obstacle_point_fun();
    //  oFile_init<< "section ahead liner222 "<< " "<<endl;
      cout<<"最后的值Speed_change = "<<Speed_change<<endl;
      msg.Speed_change_of_obstacle = Speed_change;
      oFile_init<< "state = "<<state<<"  Speed_change "<<Speed_change<<" O_index"<<Obstacle_index_through<<" x = "<<robot_pose.x<<" y = "<<robot_pose.y<<" A = "<<robot_pose.angle*57.29578<<" Continued  "<<Continued<<" relative.x "<<relative_obstacle_point3.x<<" relative.y = "<<relative_obstacle_point3.y<<endl;
      cout<< "state = "<<state<<"  Speed_change "<<Speed_change<<" O_index"<<Obstacle_index_through<<" x = "<<robot_pose.x<<" y = "<<robot_pose.y<<" A = "<<robot_pose.angle*57.29578<<" Continued  "<<Continued<<" relative.x "<<relative_obstacle_point3.x<<" relative.y = "<<relative_obstacle_point3.y<<endl;

      custom_msg_pub.publish(msg);
      loop_rate.sleep();
    }
    oFile_init.close();
        return 0;
}