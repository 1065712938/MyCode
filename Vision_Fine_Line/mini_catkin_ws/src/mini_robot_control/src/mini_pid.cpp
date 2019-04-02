#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
//#include <geometry_msgs/Twist.h>
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
#include <errno.h>

#include <jsoncpp/json/json.h> 
//#include <mini_robot_control/amcl_linear_x_ahead.h>
#include "wiringSerial.h"
#include "wiringPi.h"
using namespace std;
float Avoidance_Classification_group[10];
int Selection_action_mode = 0;
//int Arc_flag          = 0;
float usart_send_data = 0;
float Arri_Table_flag = 0;
float min_lidar_data  = 10;
float min_speed       = 0;
float speed_pre       = 0;
float Speed_change_of_obstacle = 0;
float Chose_XY_       = 0;
float Set_Point_X = 0;
float Set_Point_Y = 0;
#define Increase_speed    0
#define Uniform_speed     1
#define Decrease_speed    2
sensor_msgs::PointCloud g_Set_Points;

void auto_ration()
{
    cout<<"auto_ration"<<endl;
    ros::NodeHandle robot_stop;
    ros::Publisher send_stop = robot_stop.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   // for(int i = 0;i<5;i++)
     {
        geometry_msgs::Twist twist1;
        twist1.linear.x = 0; twist1.linear.y = 0; twist1.linear.z = 0;
        twist1.angular.x = 0; twist1.angular.y = 0; twist1.angular.z = 0.1;//angle_adjust_linear
        send_stop.publish(twist1);
     }
}


/*
   函数名：void delay_ms(int ms)
   功能  ：延迟函数 并在延迟过程中可以同时执行订阅的函数
   传入参数：ms
返回值：无
*/
void delay_ms(int ms)
{
  for(int i = 0;i<ms;i++)
  {
    for(int j=0;j<124000;j++);
    ros::spinOnce();
  }
}
/*
 函数名：void amcl_linear_motion
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:选择前进和后退 1 前进 0 后退
返回值：无
实例：amcl_linear_motion(0.1,0.25,1,1)
曾经出现的问题：加速度过大 导致机器人还没到达坐标点速度接近于0 进入死循环 
解决办法 ：Acceletare = Acceletare*0.9
*/
void amcl_linear_motion(float V0,float V1,double Distance,char flag)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 0;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   int flag_while = 0;
   flag_array[5]  = flag_array[5] + Distance-1;
   if(abs(V1)<abs(V0)) 
      Acceletare = (pow(V1,2)-pow(V0,2))/(2*Distance)*0.8;
    else
       Acceletare = (pow(V1,2)-pow(V0,2))/(2*Distance);
    cout<<"Acceletare = "<<Acceletare<<endl;
  //  cout<<"abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
    //listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
    //listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
    //pre_amcl_distance = transform_amcl_pose.getOrigin().x();

    listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
    pre_odom_distance = abs(transform_linear.getOrigin().x());
    cout<<"pre_odom_distanceKKKKKKKKK = "<<pre_odom_distance<<endl;
   while((abs(Cur_Distance-abs(Distance))>0.02)&&ros::ok()&&(flag_while<10))
   {
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        //listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        //listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
       // ROS_INFO("x = %f",listener_amcl_pose.getOrigin().x());
       // ROS_INFO("y = %f",listener_amcl_pose.getOrigin().y());
        cout << "x = "<<transform_linear.getOrigin().x() << endl ;
        q_linear.x() = transform_linear.getRotation().getX();
        q_linear.y() = transform_linear.getRotation().getY();
        q_linear.z() = transform_linear.getRotation().getZ();
        q_linear.w() = transform_linear.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);

        cout << "x = "<<transform_linear.getOrigin().x() << "  W_x = "<<transform_amcl_pose.getOrigin().x() << endl ;
        cout << "y = "<<transform_linear.getOrigin().y() << "  W_y = "<<transform_amcl_pose.getOrigin().y() << endl ;
       // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        angle_adjust_linear = PID_realize(0,euler_linear[2]*57.29578);
       // adjust_amcl_Y       = PID_realize(0,transform_amcl_pose.getOrigin().y());
        adjust_amcl_Y       = PID_realize(0,transform_linear.getOrigin().y());
        cout << "adjust_amcl_Y = "<<adjust_amcl_Y << endl;
        if (angle_adjust_linear<=-0.01281)
        angle_adjust_linear = -0.01281;
        if (angle_adjust_linear>0.01281)
        angle_adjust_linear = 0.01281;
        cout << "angle_adjust_linear = "<<angle_adjust_linear << "A = "<<euler_linear[2]*57.29578<<endl;
        cout <<"amcl.y = "<<transform_linear.getOrigin().y()<< "  adjust_amcl_Y = "<<adjust_amcl_Y<< endl;
        //cout<<"Cur_speed = "<<Cur_speed<<endl;
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
        //cout<<"end_be.toSec() = "<<end_be.toSec()<<"Acceletare = "<<Acceletare<<endl;
        Cur_speed  = Cur_speed + Acceletare*(end_be.toSec());
        if (abs(Cur_speed)>max(abs(V0),abs(V1)))
            Cur_speed = max(abs(V0),abs(V1));
        if (abs(Cur_speed)<=0.05)
            {
               Cur_speed = 0.05;
               flag_while = flag_while+1;
            }
           
        cout<<"flag_while = "<<flag_while<<endl;
        if(flag == 1) //前进
           Cur_speed = abs(Cur_speed);
        else
           Cur_speed = -abs(Cur_speed);
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angle_adjust_linear;//angle_adjust_linear -Cur_speed*0.05
        send_vel1.publish(twist);
        cout<<"Cur_speed = "<<Cur_speed<<endl;
        //Cur_Distance = Cur_Distance+abs(Cur_speed)*(end_be.toSec());
       //Cur_Distance = transform_amcl_pose.getOrigin().x()-pre_amcl_distance;
       Cur_Distance = abs(abs(transform_linear.getOrigin().x())-pre_odom_distance);
       cout<<"abs(transform_linear.getOrigin().x()) = "<<abs(transform_linear.getOrigin().x())<<"  pre_odom_distance = "<<pre_odom_distance<<endl;
        //cout<<"Cur_Distance = "<<Cur_Distance<<" Distance = "<<Distance<<endl;
        cout<<"abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-abs(Distance))<<endl;
         //cout<<"Cur_Distance = "<<Cur_Distance<<endl;
   }
     
}
/*
 函数名：void Arc_path_right_optimization
 说明 ：进行右轮速度不变的圆弧运动
 传入参数4个：
            L：两个轮子间的距离 单位cm
            speed:为圆外侧速度 前进为正 后退为负 
            Distance:所要到达的全局坐标(X或Y)
            Set_Angle：设置运行停止的角度
            change_x_y ：选择圆弧的模式
                                       1 
                                      ————                                     
                                    2|   |0   //形成一个圆弧轨迹 每个不同方向的弧线 其夹角的表达方式不同
                                     |   |    //需要根据陀螺仪的实际情况设置这个夹角的值
                                      ————
                                       3
返回值：无
例子  ：    Arc_path_right_optimization(41.0,0.2,3.3,176,1);

*/
void Arc_path_right_optimization(float L,float speed,float Distance,float Set_Angle,int change_x_y)
{
   
    ros::Rate loop_rate_Arc_path(100);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    tf::TransformListener listener_amcl_pose;
    tf::StampedTransform transform_amcl_pose;
    Eigen::Quaterniond q_Arc_path_test;
    float Rotate_1=0;
    float R_1=0;
    float Distance_diff = 0;
    float K = 0;
    float Angle_Arc = 0;
    float radian = 0;
    float adjust_Arc = 0;
    float atan_A = 0;
    float Angle_chance = 0;
    float speed_coefficient = 0;
    float K_coefficient     = 0;
    float Cur_speed  = speed;
    float Acceletare = 0; 
    float speed_inc  = 0;
    flag_array[3] = 0;
    Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
    q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
    q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
    q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
    q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
    Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
   // cout << "x= "<<transform_Arc_path_test.getOrigin().x() << endl ;
   // cout << "A= "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
    Distance_diff=Distance-transform_Arc_path_test.getOrigin().x();
    radian = 1.570796-euler_linear[2];
    L = L/100.0;
    K = L*speed*pow(sin(radian/2.0),2)/(Distance_diff+L*pow(sin(radian/2.0),2))*1.095;//左右轮子速度和车体速度的差
    cout<<"K = "<<K<<endl;
    geometry_msgs::Twist twist;
    ros::spinOnce();
 //   cout<<"Distance = "<<Distance<<"flag_array[3] = "<<flag_array[3]<<endl;
    oFile_init<< "section Arc ahead "<< " "<<endl;
    while((flag_array[3]==0)&&ros::ok())
    {
        ros::Time begin = ros::Time::now();
        run_get_pose();
        Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
        q_Arc_path_test.x() = transform_amcl_pose.getRotation().getX();
        q_Arc_path_test.y() = transform_amcl_pose.getRotation().getY();
        q_Arc_path_test.z() = transform_amcl_pose.getRotation().getZ();
        q_Arc_path_test.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
       // cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Arc = euler_linear[2]*57.29578;
        ros::Time end1 = ros::Time::now();
        ros::Duration end_be = end1 - begin;
        if(Selection_action_mode == 0)
        {
            Acceletare = (pow(speed,2)-pow(Cur_speed,2));
            Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
            speed_inc = 0;
            speed_coefficient = 1;
            K_coefficient     = 1;
        }
        else if(Selection_action_mode == 1)
        {
              //  ros::Time end = ros::Time::now();
              // ros::Duration end_be = end - begin;
                Cur_speed = Cur_speed - (Cur_speed*Cur_speed - 0)/(2.0*abs(min_lidar_data-0.5))*end_be.toSec();//*2
                if(Cur_speed<0.05)
                {
                  Cur_speed = 0.05;
                  speed_inc = speed/3;
                }
                speed_coefficient = 1;
                K_coefficient     = 1;
        }
        else{ 
                Cur_speed = 0;
                speed_inc =speed/2;
                speed_coefficient = 0;
                K_coefficient     = 0;
            }
        switch(change_x_y)
        {
            case 0 :
                cout<< "You entered change_x_y = 0 \n";
                radian = 1.570796-euler_linear[2];
                Distance_diff=Distance-transform_amcl_pose.getOrigin().x();
                Angle_chance = Set_Angle - Angle_Arc;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                if(K>=0.1)    K = 0.1;
                cout<<"K = "<<K<<endl;
                break;
            case 1 :
                cout<< "You entered change_x_y = 1 \n";
                radian = 3.1415926-euler_linear[2];
                Distance_diff=Distance-transform_amcl_pose.getOrigin().y();
                Angle_chance = Set_Angle - Angle_Arc;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed*1.095-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                cout<<"K = "<<K<<endl;
                break;
            case 2 :
               cout<< "You entered change_x_y = 2 \n";
               radian =abs(euler_linear[2])- 1.570796;
               cout<<"W_x = "<<transform_amcl_pose.getOrigin().x()<<"  Distance ="<<Distance<<endl;
               Distance_diff=-Distance+transform_amcl_pose.getOrigin().x();
               cout<<"abs(Distance_diff )= "<<abs(Distance-transform_amcl_pose.getOrigin().x())<<endl;
               Angle_chance = abs(Angle_Arc) + Set_Angle;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed*1.095-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                cout<<"K = "<<K<<endl;  
                break;
            case 3 :
                cout<< "You entered change_x_y = 3 \n";
                radian = -euler_linear[2];
                cout<<"Y = "<<transform_amcl_pose.getOrigin().y()<<" Distance "<<Distance<<endl;
                Distance_diff=transform_amcl_pose.getOrigin().y()-Distance;
                Angle_chance = Set_Angle - Angle_Arc;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed*1.095-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                if(K>=0.1)    K = 0.1;
                cout<<"K = "<<K<<endl;
                break;
            default:
                cout << "You did not enter 0, 1, 2 ,3!\n";
        }
      if(Angle_chance>=0)
        {

                flag_array[3] = 0;
                geometry_msgs::Twist twist;
                /*
                if(Selection_action_mode == 0){
                    speed_coefficient = 1;
                    K_coefficient    = 1;
                }
                else
                {
 //                   speed_coefficient = 0;
 //                   K_coefficient     = 0;
                }
                */
                min_speed = K;//(0.6-2*K + 0.6)/2 = 0.6 - K
                cout<<"Cur_speed = "<<Cur_speed <<"Cur_speed-2K = "<<Cur_speed-2*K<<endl;
                twist.linear.x = (Cur_speed-K)*speed_coefficient;twist.linear.y = 0; twist.linear.z = 0;
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =(K/L)*K_coefficient;//K
                send_vel1_Arc_path.publish(twist);
        }
        else
        {
            flag_array[3] = 1;
        }
        loop_rate_Arc_path.sleep();
       // ros::Time end = ros::Time::now();
       // ros::Duration end_be = end - begin;
     //   cout<<"end_be.toSec() = "<<end_be.toSec()<<endl;
    }
}






/*
 函数名：void Arc_path_right_optimization
 说明 ：进行右轮速度不变的圆弧运动
 传入参数4个：
            L：两个轮子间的距离 单位cm
            speed:为圆外侧速度 前进为正 后退为负 
            Distance:所要到达的全局坐标(X或Y)
            Set_Angle：设置运行停止的角度
            change_x_y ：选择圆弧的模式
                                       1 
                                      ————                                     
                                    2|   |0   //形成一个圆弧轨迹 每个不同方向的弧线 其夹角的表达方式不同
                                     |   |    //需要根据陀螺仪的实际情况设置这个夹角的值
                                      ————
                                       3
返回值：无
例子  ：    Arc_path_right_optimization(41.0,0.2,3.3,176,1);

*/
void Arc_path_right_optimization_avoidance(float L,float speed,float Distance,float Set_Angle,int change_x_y)
{
   
    ros::Rate loop_rate_Arc_path(100);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    ros::Publisher send_pose = Arc_path.advertise<geometry_msgs::Twist>("send_pose",2);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    tf::TransformListener listener_amcl_pose;
    tf::StampedTransform transform_amcl_pose;
    Eigen::Quaterniond q_Arc_path_test;
    float Rotate_1=0;
    float R_1=0;
    float Distance_diff = 0;
    float K = 0;
    float Angle_Arc = 0;
    float radian = 0;
    float adjust_Arc = 0;
    float atan_A = 0;
    float Angle_chance = 0;
    float speed_coefficient = 0;
    float K_coefficient     = 0;
    float Cur_speed  = speed;
    float Acceletare = 0; 
    float speed_inc  = 0;
    flag_array[3] = 0;
    Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
    q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
    q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
    q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
    q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
    Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
   // cout << "x= "<<transform_Arc_path_test.getOrigin().x() << endl ;
   // cout << "A= "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
    Distance_diff=Distance-transform_Arc_path_test.getOrigin().x();
    radian = 1.570796-euler_linear[2];
    L = L/100.0;
    K = L*speed*pow(sin(radian/2.0),2)/(Distance_diff+L*pow(sin(radian/2.0),2))*1.095;//左右轮子速度和车体速度的差
    cout<<"K = "<<K<<endl;
    geometry_msgs::Twist twist;
    ros::spinOnce();
 //   cout<<"Distance = "<<Distance<<"flag_array[3] = "<<flag_array[3]<<endl;
    oFile_init<< "section Arc ahead "<< " "<<endl;
    while((flag_array[3]==0)&&ros::ok())
    {
        ros::Time begin = ros::Time::now();
        run_get_pose();
        Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
        q_Arc_path_test.x() = transform_amcl_pose.getRotation().getX();
        q_Arc_path_test.y() = transform_amcl_pose.getRotation().getY();
        q_Arc_path_test.z() = transform_amcl_pose.getRotation().getZ();
        q_Arc_path_test.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
       // cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Arc = euler_linear[2]*57.29578;
        ros::Time end1 = ros::Time::now();
        ros::Duration end_be = end1 - begin;
        //Selection_action_mode = 0;
        if(Selection_action_mode == 0)
        {
            Acceletare = (pow(speed,2)-pow(Cur_speed,2));
            Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
            speed_inc = 0;
            speed_coefficient = 1;
            K_coefficient     = 1;
        }
        else if(Selection_action_mode == 1)
        {
              //  ros::Time end = ros::Time::now();
              // ros::Duration end_be = end - begin;
                Cur_speed = Cur_speed - (Cur_speed*Cur_speed - 0)/(2.0*abs(min_lidar_data-0.5))*end_be.toSec();//*2
                if(Cur_speed<0.05)
                {
                  Cur_speed = 0.05;
                  speed_inc = speed/3;
                }
                speed_coefficient = 1;
                K_coefficient     = 1;
        }
        else{ 
                Cur_speed = 0;
                speed_inc =speed/2;
                speed_coefficient = 0;
                K_coefficient     = 0;
            }
        switch(change_x_y)
        {
            case 0 :
                cout<< "You entered change_x_y = 0 \n";
                radian = 1.570796-euler_linear[2];
                Distance_diff=Distance-transform_amcl_pose.getOrigin().x();
                Angle_chance = Set_Angle - Angle_Arc;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                if(K>=0.1)    K = 0.1;
                Chose_XY_       = 2;
                cout<<"K = "<<K<<endl;
                break;
            case 1 :
                cout<< "You entered change_x_y = 1 \n";
                radian = 3.1415926-euler_linear[2];
                Distance_diff=Distance-transform_amcl_pose.getOrigin().y();
                Angle_chance = Set_Angle - Angle_Arc;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed*1.095-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                cout<<"K = "<<K<<endl;
                break;
            case 2 :
               cout<< "You entered change_x_y = 2 \n";
               radian =abs(euler_linear[2])- 1.570796;
               cout<<"W_x = "<<transform_amcl_pose.getOrigin().x()<<"  Distance ="<<Distance<<endl;
               Distance_diff=-Distance+transform_amcl_pose.getOrigin().x();
               cout<<"abs(Distance_diff )= "<<abs(Distance-transform_amcl_pose.getOrigin().x())<<endl;
               Angle_chance = abs(Angle_Arc) + Set_Angle;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed*1.095-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                cout<<"K = "<<K<<endl;  
                break;
            case 3 :
                cout<< "You entered change_x_y = 3 \n";
                radian = -euler_linear[2];
                cout<<"Y = "<<transform_amcl_pose.getOrigin().y()<<" Distance "<<Distance<<endl;
                Distance_diff=transform_amcl_pose.getOrigin().y()-Distance;
                Angle_chance = Set_Angle - Angle_Arc;
                if(Distance_diff<0.02) Distance_diff = 0.02;
               // cout<<"Distance_diff = "<<Distance_diff<<" change_x_y="<<change_x_y<<" radian="<<radian<<endl;
                K = L*Cur_speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
                if(K>=Cur_speed*1.095-0.08) K = Cur_speed-0.08;
                if(K<=0.007)    K = 0.007;
                if(K>=0.1)    K = 0.1;
                cout<<"K = "<<K<<endl;
                break;
            default:
                cout << "You did not enter 0, 1, 2 ,3!\n";
        }
      if(Angle_chance>=0)
        {

                flag_array[3] = 0;
                geometry_msgs::Twist twist;
                /*
                if(Selection_action_mode == 0){
                    speed_coefficient = 1;
                    K_coefficient    = 1;
                }
                else
                {
 //                   speed_coefficient = 0;
 //                   K_coefficient     = 0;
                }
                */
                min_speed = K;//(0.6-2*K + 0.6)/2 = 0.6 - K
                cout<<"Cur_speed = "<<Cur_speed <<"Cur_speed-2K = "<<Cur_speed-2*K<<endl;
                twist.linear.x = (Cur_speed-K)*speed_coefficient;
                twist.linear.y = Chose_XY_;
                twist.linear.z = euler_linear[2];
                twist.angular.x = transform_amcl_pose.getOrigin().x(); 
                twist.angular.y = transform_amcl_pose.getOrigin().y(); 
                twist.angular.z =(K/L)*K_coefficient;//K All_adjust
                send_vel1_Arc_path.publish(twist);
                send_pose.publish(twist);
        }
        else
        {
            flag_array[3] = 1;
        }
        oFile_init<< "Selection_action_mode "<< Selection_action_mode<<" Chose_XY_ "<<Chose_XY_<<endl;
        loop_rate_Arc_path.sleep();
       // ros::Time end = ros::Time::now();
       // ros::Duration end_be = end - begin;
     //   cout<<"end_be.toSec() = "<<end_be.toSec()<<endl;
    }
}


/*
 函数名：void Arc_path_right_back_optimization
 传入参数4个：
            R:圆弧的半径
            L：两个轮子间的距离 单位cm
            speed:为圆外侧速度 前进为正 后退为负 
            Distance:弧线端点的直线距离
返回值：无
例子  ：    Arc_path_right_back_optimization(0.707*0.75,41.0,0.2,0.45*3.141*0.5,85);
*/
void Arc_path_right_back_optimization(float R,float L,float speed,float Distance,float Set_Angle)
{
    cout<<"圆弧外侧速度不变的弧线运动 返回"<<endl;
    ros::Rate loop_rate_Arc_path(100);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    tf::TransformListener listener_amcl_pose;
    tf::StampedTransform transform_amcl_pose;
    Eigen::Quaterniond q_Arc_path_test;
    float Rotate_1=0;
    float R_1=0;
    float Distance_diff = 0;
    float K = 0;
    float Angle_Arc = 0;
    float radian = 0;
    float adjust_Arc = 0;
    float atan_A = 0;
    flag_array[3] = 0;
   
    Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
    q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
    q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
    q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
    q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
    
    Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
   // cout << "x= "<<transform_Arc_path_test.getOrigin().x() << endl ;
   // cout << "A= "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
    Distance_diff=Distance-transform_Arc_path_test.getOrigin().x();
    radian = 1.570796-euler_linear[2];
    L = L/100.0;
    K = L*speed*pow(sin(radian/2.0),2)/(Distance_diff+L*pow(sin(radian/2.0),2))*1.095;//左右轮子速度和车体速度的差
    cout<<"K = "<<K<<endl;
    geometry_msgs::Twist twist;
    ros::spinOnce();
 //   cout<<"Distance = "<<Distance<<"flag_array[3] = "<<flag_array[3]<<endl;
 //   oFile_init<< "section Arc BACK "<< " "<<endl;
    while((flag_array[3]==0)&&ros::ok())
    {
        ros::Time begin1 = ros::Time::now();
        run_get_pose();
        Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
        q_Arc_path_test.x() = transform_amcl_pose.getRotation().getX();
        q_Arc_path_test.y() = transform_amcl_pose.getRotation().getY();
        q_Arc_path_test.z() = transform_amcl_pose.getRotation().getZ();
        q_Arc_path_test.w() = transform_amcl_pose.getRotation().getW();

        //q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
        //q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
        //q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
       // q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
       // cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
     //   cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Arc = euler_linear[2]*57.29578;
       // cout<<"Angle_Arc = "<<Angle_Arc<<endl;
        //speed = speed - abs(Set_Angle-Angle_Arc)*0.002;
    //    cout<<"speed = "<<speed<<endl;
     //   oFile_init<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<" "<<"A"<<" "<<Angle_Arc<<endl;
        radian = 3.1415926+euler_linear[2];
        //Distance_diff=Distance-transform_Arc_path_test.getOrigin().x();
        Distance_diff=Distance-transform_amcl_pose.getOrigin().y();
        if(Distance_diff>=0)
          Distance_diff = 0.01;
        Distance_diff = abs(Distance_diff);
       // cout<<"Distance_diff = "<<Distance_diff<<endl;
        K = L*speed*pow(sin(radian/2.0),2)/(Distance_diff*1.3+L*pow(sin(radian/2.0),2));//左右轮子速度和车体速度的差
        K = -K;
      //  cout<<"abs(K) = "<<abs(K)<<endl;
        if(abs(K)>speed/2.0) //K地为负数
        {
          K = -speed/2.0+0.001;
        //  cout<<"K = speed-0.001 = "<<K<<endl;
        }
           
        cout<<"K = "<<K<<endl;
        //K = L*speed/(2*R+L);
       //amcl_get_pose();//打印AMCL坐标值  if((Angle_Arc<=Set_Angle)||(Angle_Arc>90))

       if((Angle_Arc<=-172)||(Angle_Arc>90))
        {
            flag_array[3] = 1;
        }
        else
        {
            flag_array[3] = 0;
            geometry_msgs::Twist twist;
            twist.linear.x =speed+K;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K
            send_vel1_Arc_path.publish(twist);
        }
    }
    cout<<"Angle_Arc = "<<Angle_Arc<<endl;
}
/*
 函数名：void amcl_linear_Y_ahead
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:选择前进和后退 1 前进 0 后退
返回值：无
实例：amcl_linear_motion(0.1,0.25,1,1,2.0)
曾经出现的问题：加速度过大 导致机器人还没到达坐标点速度接近于0 进入死循环 
解决办法 ：Acceletare = Acceletare*0.9
*/
void amcl_linear_Y_ahead(float V0,float V1,double Distance,char flag,float set_Pid)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 0;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float All_adjust    = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   int flag_while = 0;
   listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
   listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
    pre_odom_distance = abs(transform_linear.getOrigin().x());
   //if(abs(V1)<abs(V0)) 
   //   Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance))*0.8;
   // else
    //   Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance));
    //oFile_init<< "section amcl_linear_Y_ahead "<< " "<<endl;
  //  cout<<"Cur_speed_VO = "<<Cur_speed<<endl;
   while((Cur_Distance<Distance)&&ros::ok()&&(flag_while<10))
   {
    //    cout<<"(abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
     //   cout << "x = "<<transform_linear.getOrigin().x() << endl ;
        q_linear.x() = transform_amcl_pose.getRotation().getX();
        q_linear.y() = transform_amcl_pose.getRotation().getY();
        q_linear.z() = transform_amcl_pose.getRotation().getZ();
        q_linear.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);

       // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
      //  pid.Kp=0.01;//角度直线参数
      //  pid.Ki=0.01;
      //  pid.Kd=0.005;
      // angle_adjust_linear = -PID_realize(0,euler_linear[2]*57.29578);
       pid.Kp=0.16;//角度直线参数
       pid.Ki=0.0;
       pid.Kd=0.2;
       //adjust_amcl_Y       = -PID_realize(0,1);
      // adjust_amcl_Y       = -PID_realize(set_Pid,transform_amcl_pose.getOrigin().x());
        //All_adjust = 0.8*angle_adjust_linear + adjust_amcl_Y*1.3-Cur_speed*0.04;
        All_adjust = adjust_amcl_Y;
        cout << "All_adjust = "<<All_adjust << endl;
        if (All_adjust<=-0.019) All_adjust = -0.019;
        if (All_adjust>0.019)  All_adjust = 0.019;
         
        
        cout << "angle_adjust_linear = "<<angle_adjust_linear << "  A = "<<euler_linear[2]*57.29578<<endl;
        cout <<"amcl.y = "<<transform_amcl_pose.getOrigin().y()<< "  adjust_amcl_Y = "<<adjust_amcl_Y<< endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
        Cur_speed  = Cur_speed + Acceletare*(end_be.toSec());
        Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*(Distance - Cur_Distance));
        if (abs(Cur_speed)>max(abs(V0),abs(V1)))
            Cur_speed = max(abs(V0),abs(V1));
        if (abs(Cur_speed)<=0.05)//0.05
            {
               Cur_speed = 0.05;
            //   flag_while = flag_while+1;
            }
           
       // cout<<"flag_while = "<<flag_while<<endl;
        if(flag == 1) //前进
           Cur_speed = abs(Cur_speed);
        else
           Cur_speed = -abs(Cur_speed);
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;//All_adjust
        send_vel1.publish(twist);
        cout<<"Cur_speed = "<<Cur_speed<<endl;
        //Cur_Distance = transform_linear.getOrigin().x();//里程坐标
        Cur_Distance = transform_amcl_pose.getOrigin().y();//amcl定位坐标
        cout << "W_x = "<<transform_amcl_pose.getOrigin().x() << "  W_y = "<<transform_amcl_pose.getOrigin().y() << endl ;
//        cout << "y = "<<transform_linear.getOrigin().y() << "  W_y = "<<transform_amcl_pose.getOrigin().y() << endl ;
//        oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<< " "<<"A"<<" "<<euler_linear[2]*57.29578<<" "<< "O_X"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<"g_odom_x"<<" "<<g_odom_x<< " "<<"g_odom_y"<<" "<<g_odom_y<<endl;
   }
    
}

/*
 函数名：void amcl_linear_ahead
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:选择前进和后退 1 前进 0 后退
返回值：无
实例：amcl_linear_ahead(0.1,0.25,1,1)
曾经出现的问题：加速度过大 导致机器人还没到达坐标点速度接近于0 进入死循环 
解决办法 ：Acceletare = Acceletare*0.9
*/
void amcl_linear_ahead(float V0,float V1,double Distance,char flag)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   ros::Rate rate(10.0);
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 0;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float All_adjust    = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   float current_yaw = 0;
   int flag_while = 0;
   listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
   listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
   pre_odom_distance = abs(transform_linear.getOrigin().x());
   if(abs(V1)<abs(V0)) 
      Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance))*0.8;
    else
       Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance));
   while((Cur_Distance<Distance)&&ros::ok()&&(flag_while<10))
   {
    //    tf::StampedTransform transform_amcl_pose;
       // cout<<"(abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
        ros::Time begin = ros::Time::now();
        try{
              listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
              listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
             // listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(3.0));
             // listener_amcl_pose.lookupTransform("/map", "/base_link",  ros::Time::now(), transform_amcl_pose);
              listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
              listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
          }
        catch(tf::TransformException& ex)
         {
           ROS_ERROR("nimei Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
           ros::Duration(1.0).sleep();
           continue;
         }

        //cout << "x = "<<transform_linear.getOrigin().x() << endl ;
     //   cout<< "W_x = "<<transform_amcl_pose.getOrigin().x() <<" g_odom_x = "<<g_odom_x<<endl;
        q_linear.x() = transform_linear.getRotation().getX();
        q_linear.y() = transform_linear.getRotation().getY();
        q_linear.z() = transform_linear.getRotation().getZ();
        q_linear.w() = transform_linear.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
        current_yaw = euler_linear[2]*57.29578;
       // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
      //  pid.Kp=0.01;//角度直线参数
      //  pid.Ki=0.01;
      //  pid.Kd=0.005;
      // angle_adjust_linear = -PID_realize(0,euler_linear[2]*57.29578);
       pid.Kp=0.16;//角度直线参数
       pid.Ki=0.15;
       pid.Kd=0.08;
       //adjust_amcl_Y       = -PID_realize(0,1);
       adjust_amcl_Y       = PID_realize(0,transform_amcl_pose.getOrigin().y());
        //All_adjust = 0.8*angle_adjust_linear + adjust_amcl_Y*1.3-Cur_speed*0.04;
        All_adjust = adjust_amcl_Y;  
     //   cout << "angle_adjust_linear = "<<angle_adjust_linear << "  A = "<<euler_linear[2]*57.29578<<endl;
     //   cout <<"amcl.y = "<<transform_amcl_pose.getOrigin().y()<< "  adjust_amcl_Y = "<<adjust_amcl_Y<< endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
        Cur_speed  = Cur_speed + Acceletare*(end_be.toSec());
        if (abs(Cur_speed)>max(abs(V0),abs(V1)))
            Cur_speed = max(abs(V0),abs(V1));
        if (abs(Cur_speed)<=0.05)
            {
               Cur_speed = 0.05;
               flag_while = flag_while+1;
            }
           
       // cout<<"flag_while = "<<flag_while<<endl;
        if(flag == 1) //前进
           Cur_speed = abs(Cur_speed);
        else
           Cur_speed = -abs(Cur_speed);
        if((abs(current_yaw)>4)&&(abs(transform_amcl_pose.getOrigin().y())<0.04))//0.04
          {
            if(((transform_amcl_pose.getOrigin().y()>0)&&(current_yaw<0))||((transform_amcl_pose.getOrigin().y()<0)&&(current_yaw>0)))
              {
                 All_adjust = Cur_speed*sin(euler_linear[2]/2)*sin(euler_linear[2]/2)/transform_amcl_pose.getOrigin().y();
                // cout<<"current_yaw = "<<current_yaw<<" euler_linear[2] = "<<euler_linear[2]<<"transform_amcl_pose.getOrigin().y() = "<<transform_amcl_pose.getOrigin().y()<<"All_adjust = "<<All_adjust<<endl;
              }
          }
       // cout << "All_adjust = "<<All_adjust << endl;
        if (All_adjust<=-0.025) All_adjust = -0.025;//0.019
        if (All_adjust>0.025)  All_adjust = 0.025;
        //避障时可以在此处加入全局变量 判断发原来的速度还是发停止命令
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =All_adjust;//angle_adjust_linear+adjust_amcl_Yangle_adjust_linear -Cur_speed*0.05
        send_vel1.publish(twist);
     //   cout<<"Cur_speed = "<<Cur_speed<<endl;
        Cur_Distance = transform_linear.getOrigin().x();//里程坐标
       // Cur_Distance = transform_amcl_pose.getOrigin().x();//amcl定位坐标
        cout << "x = "<<transform_linear.getOrigin().x() << "  W_x = "<<transform_amcl_pose.getOrigin().x() << endl ;
       // cout << "y = "<<transform_linear.getOrigin().y() << "  W_y = "<<transform_amcl_pose.getOrigin().y() << endl ;
        oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<< " "<<"A"<<" "<<euler_linear[2]*57.29578<<" "<< "O_X"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<"g_odom_x"<<" "<<g_odom_x<< " "<<"g_odom_y"<<" "<<g_odom_y<<endl;
      rate.sleep();
   }
    oFile_init<< "section ahead liner "<< " "<<endl;
}



/*
 函数名：void amcl_linear_ahead_2
 说明  ：void amcl_linear_ahead 的改进版
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:0 ：在X为正方向运行 1：X为负方向运行
            state:选择状态 0 :加速 1:匀速  2:减速
            pid_value 设置PID的目标值
返回值：无
实例：amcl_linear_ahead(0.1,0.25,1,1)
曾经出现的问题：加速度过大 导致机器人还没到达坐标点速度接近于0 进入死循环 
解决办法 ：Acceletare = Acceletare*0.9
*/
void amcl_linear_ahead_2(float V0,float V1,double Distance,char flag,char state,float pid_value)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   ros::Rate rate(10.0);
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 0;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float All_adjust    = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   float current_yaw = 0;
   int flag_while = 0;
   float dirction_flag = 0;
   float speed_inc     = 0;
   while((dirction_flag>=0)&&ros::ok())
   {  
        ros::Time begin = ros::Time::now();
        try{
        //      listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        //      listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
              listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
              listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
          }
        catch(tf::TransformException& ex)
         {
           ROS_ERROR("nimei Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
           ros::Duration(1.0).sleep();
           continue;
         }
        q_linear.x() = transform_amcl_pose.getRotation().getX();
        q_linear.y() = transform_amcl_pose.getRotation().getY();
        q_linear.z() = transform_amcl_pose.getRotation().getZ();
        q_linear.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
        current_yaw = euler_linear[2]*57.29578;
        pid.Kp=0.16;//角度直线参数
        pid.Ki=0.15;
        pid.Kd=0.08;
        adjust_amcl_Y       = PID_realize(pid_value,transform_amcl_pose.getOrigin().y());
        All_adjust = adjust_amcl_Y;  
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
        if(Selection_action_mode == 0){ //不存在障碍物
             All_adjust = All_adjust;
             if(state == 0) //0 表示加速状态
             {
                Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
                Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*(Distance - Cur_Distance));
             }
             else
              {
                Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*(Distance - Cur_Distance));
                Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
              }
             speed_inc = 0;
          }
        else if(Selection_action_mode == 1) //表示离障碍物较远
        {
             ros::Time end = ros::Time::now();
             ros::Duration end_be = end - begin;
             Cur_speed = Cur_speed - (Cur_speed*Cur_speed - 0)/(2.0*abs(min_lidar_data-0.5))*end_be.toSec();//*2
            if(Cur_speed<0.05)
            Cur_speed = 0.05;
            //speed_inc = V0/2;
           //cout<<" Cur_speed = "<< Cur_speed<<" min_lidar_data = "<<min_lidar_data<<"  inc_v = "<<(Cur_speed*Cur_speed - V0*V0)/(2.0*min_lidar_data)*end_be.toSec()<<"  end_be.toSec() ="<<end_be.toSec()<<endl;
        }
        else{ //离障碍物很近 需要及停
              Cur_speed  =  0;
              All_adjust =  0;
              speed_inc  =  V0;
              Send_stop(); //完全停止后 在
              ros::Duration(3).sleep(); 
        }
        if (abs(Cur_speed)>max(abs(V0),abs(V1)))
             Cur_speed = max(abs(V0),abs(V1));
        if((abs(current_yaw)>4)&&(abs(transform_amcl_pose.getOrigin().y())<0.04))//0.04
          {
            if(((transform_amcl_pose.getOrigin().y()>0)&&(current_yaw<0))||((transform_amcl_pose.getOrigin().y()<0)&&(current_yaw>0)))
              {
                 All_adjust = Cur_speed*sin(euler_linear[2]/2)*sin(euler_linear[2]/2)/transform_amcl_pose.getOrigin().y();
                // cout<<"current_yaw = "<<current_yaw<<" euler_linear[2] = "<<euler_linear[2]<<"transform_amcl_pose.getOrigin().y() = "<<transform_amcl_pose.getOrigin().y()<<"All_adjust = "<<All_adjust<<endl;
              }
          }
        if (All_adjust<=-0.025) All_adjust = -0.025;//0.019
        if (All_adjust>0.025)  All_adjust = 0.025;
        //避障时可以在此处加入全局变量 判断发原来的速度还是发停止命令
     //   cout<<"Selection_action_mode = "<<Selection_action_mode<<endl;
     //   if(state == 0)
      //    Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*(Distance - Cur_Distance));
      //  else
      //    Acceletare = (pow(V1,2)-pow(Cur_speed,2));
    //    cout<<"Acceletare = "<<Acceletare<<"  Cur_speed = "<<Cur_speed<<endl;
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;//All_adjust
        send_vel1.publish(twist);
        //Cur_Distance = transform_linear.getOrigin().x();//里程坐标
        Cur_Distance = transform_amcl_pose.getOrigin().x();//amcl定位坐标
        cout << "amcl_x = "<<transform_amcl_pose.getOrigin().x() << " amcl_y = "<<transform_amcl_pose.getOrigin().y() << "Cur_speed ="<<Cur_speed<<endl ;
        if(flag ==0) dirction_flag = Distance - Cur_Distance;
        else  dirction_flag = Cur_Distance - Distance; 
        cout<<"dirction_flag = "<<dirction_flag<<endl;
      //  oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<< " "<<"A"<<" "<<euler_linear[2]*57.29578<<" "<< "O_X"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<"g_odom_x"<<" "<<g_odom_x<< " "<<"g_odom_y"<<" "<<g_odom_y<<endl;
      //  rate.sleep();
   }
    //oFile_init<< "section ahead liner "<< " "<<endl;
}
/*
 函数名：void amcl_linear_ahead_avoidance
 说明  ：用于避障测试
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:0 ：在X为正方向运行 1：X为负方向运行
            state:选择状态 0 :加速 1:匀速  2:减速
            pid_value 设置PID的目标值
返回值：无
实例：amcl_linear_ahead(0.1,0.25,1,1)
曾经出现的问题：加速度过大 导致机器人还没到达坐标点速度接近于0 进入死循环 
解决办法 ：Acceletare = Acceletare*0.9
*/
void amcl_linear_ahead_avoidance(float V0,float V1,double Distance,char flag,char state,float pid_value)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   ros::Publisher send_pose = n1.advertise<geometry_msgs::Twist>("send_pose",2);//改为1

   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   ros::Rate rate(10.0);
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 0;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float All_adjust    = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   float current_yaw = 0;
   int flag_while = 0;
   float dirction_flag = 0;
   float speed_inc     = 0;
   while((dirction_flag>=0)&&ros::ok())
   {  
        ros::Time begin = ros::Time::now();
        try{
        //      listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        //      listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
              listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
              listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
          }
        catch(tf::TransformException& ex)
         {
           ROS_ERROR("nimei Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
           ros::Duration(1.0).sleep();
           continue;
         }
        q_linear.x() = transform_amcl_pose.getRotation().getX();
        q_linear.y() = transform_amcl_pose.getRotation().getY();
        q_linear.z() = transform_amcl_pose.getRotation().getZ();
        q_linear.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
        current_yaw = euler_linear[2]*57.29578;
        pid.Kp=0.16;//角度直线参数
        pid.Ki=0.15;
        pid.Kd=0.08;
        adjust_amcl_Y       = PID_realize(pid_value,transform_amcl_pose.getOrigin().y());
        All_adjust = adjust_amcl_Y;  
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
       // Selection_action_mode = 0;//仅测试
       cout<<"OOOKKKKKSelection_action_mode = "<<Selection_action_mode<<endl;
        if(Selection_action_mode == 0){ //不存在障碍物
             All_adjust = All_adjust;
             if(state == 0) //0 表示加速状态
             {        
                Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
                Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*(Distance - Cur_Distance));
                cout<<"Acceletare = "<<Acceletare<<"  end_time = "<<end_be.toSec()<<endl;
             }
             else
              {
                //在匀速阶段要判断 Distance - Cur_Distance 值过大 考虑给一个固定值
                Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*(Distance - Cur_Distance));
                Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
              }
             speed_inc = 0;
            speed_pre = Cur_speed;
          }
        else if(Selection_action_mode == 1) //表示离障碍物较远
        {
             ros::Time end = ros::Time::now();
             ros::Duration end_be = end - begin;
             if(Cur_speed>speed_pre)
               Cur_speed = speed_pre;//全局变量 
             float Acc = (Cur_speed*Cur_speed - 0.2*0.2)/(2.0*2);
            // cout<<"Cur_speed = "<<Cur_speed<<"  Acc1 = "<<Acc<<endl;
             Cur_speed = Cur_speed - Acc*end_be.toSec();//end_be.toSec() = 0.1
            // if(Cur_speed<0.05)
            // Cur_speed = 0.05;
            if(Cur_speed<0.2)
             Cur_speed = 0.2;
            speed_pre = Cur_speed;
            //All_adjust = All_adjust*10;
            //speed_inc = V0/2;
           //  cout<<" Cur_speed11 = "<< Cur_speed<<" min_lidar_data = "<<min_lidar_data<<"  inc_v = "<<(Cur_speed*Cur_speed - 0)/(2.0*min_lidar_data)*end_be.toSec()<<"  end_be.toSec() ="<<end_be.toSec()<<endl;
        }
        else{ //离障碍物很近 需要及停
              Buffer_emergency_stop(Cur_speed,5);
              Cur_speed  =  0;
              All_adjust =  0;
              speed_inc  =  V0;
             // Send_stop(); //完全停止后 在
             // ros::Duration(3).sleep(); 
              cout<<"state = 2 停止 = "<<Selection_action_mode<<"Speed_change_of_obstacle = "<<Speed_change_of_obstacle<<endl;
        }
        if (abs(Cur_speed)>max(abs(V0),abs(V1)))
        {
          Cur_speed = max(abs(V0),abs(V1));
          cout<<"OOOOOOOKKKKKKKKKKCur_speed = "<<Cur_speed<<endl;
        }
        int arc_in_flag = 1;
        if((abs(current_yaw)>4)&&(abs(transform_amcl_pose.getOrigin().y())<0.1))//0.04
          {
            if(((transform_amcl_pose.getOrigin().y()>0)&&(current_yaw<0))||((transform_amcl_pose.getOrigin().y()<0)&&(current_yaw>0)))
              {
                 All_adjust = Cur_speed*sin(euler_linear[2]/2)*sin(euler_linear[2]/2)/transform_amcl_pose.getOrigin().y();
                // cout<<"current_yaw = "<<current_yaw<<" euler_linear[2] = "<<euler_linear[2]<<"transform_amcl_pose.getOrigin().y() = "<<transform_amcl_pose.getOrigin().y()<<"All_adjust = "<<All_adjust<<endl;
                 arc_in_flag = 2;
              }
          }
        if (All_adjust<=-0.085) All_adjust = -0.085;//0.019
        if (All_adjust>0.085)  All_adjust = 0.085;
        //避障时可以在此处加入全局变量 判断发原来的速度还是发停止命令
       //不使用避障时屏蔽下列代码
        
        if(Speed_change_of_obstacle == 0){
           All_adjust = All_adjust;
        }
        else{
            if(Selection_action_mode == 1)
            Cur_speed = 0.2;
            cout<<"  Cur_speed = "<<Cur_speed<<" Speed_change_of_obstacle"<<Speed_change_of_obstacle<<endl;
            All_adjust = Speed_change_of_obstacle;
        }
        oFile_init<<"mode "<<Selection_action_mode<<" Cur_speed "<<Cur_speed<<" All_adjust "<<All_adjust<<" arc_in_flag "<<arc_in_flag<<" Speed_ob "<<Speed_change_of_obstacle<<" x "<<transform_amcl_pose.getOrigin().x()<<" y "<<transform_amcl_pose.getOrigin().y()<<" current_yaw "<<current_yaw<<endl;
        cout<<"  All_adjust = "<<All_adjust<<"  Cur_speed = "<<Cur_speed<<endl;
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = euler_linear[2];
        twist.angular.x = transform_amcl_pose.getOrigin().x();
        twist.angular.y = transform_amcl_pose.getOrigin().y();//
        twist.angular.z =All_adjust;//All_adjust
        send_vel1.publish(twist);
        send_pose.publish(twist);
       // Cur_speed  = V0;//暂时先这样设置 还需要配合其他(例：加速运动) 做出修改
        cout<<"  X = "<<transform_amcl_pose.getOrigin().x()<<" Y = "<<transform_amcl_pose.getOrigin().y()<<endl;
        //Cur_Distance = transform_linear.getOrigin().x();//里程坐标
        Cur_Distance = transform_amcl_pose.getOrigin().x();//amcl定位坐标
     //   cout << "amcl_x = "<<transform_amcl_pose.getOrigin().x() << " amcl_y = "<<transform_amcl_pose.getOrigin().y() << "Cur_speed ="<<Cur_speed<<endl ;
        if(flag ==0) dirction_flag = Distance - Cur_Distance;
        else  dirction_flag = Cur_Distance - Distance; 
        cout<<"dirction_flag = "<<dirction_flag<<" Cur_Distance = "<<Cur_Distance<<endl;
      //  oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<< " "<<"A"<<" "<<euler_linear[2]*57.29578<<" "<< "O_X"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<"g_odom_x"<<" "<<g_odom_x<< " "<<"g_odom_y"<<" "<<g_odom_y<<endl;
      //  rate.sleep();
   }
    //oFile_init<< "section ahead liner "<< " "<<endl;
}



/*
 函数名：void amcl_linear_ahead_avoidance
 说明  ：用于避障测试
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:0 ：在X为正方向运行 1：X为负方向运行
            state:选择状态 0 :加速 1:匀速  2:减速
            pid_value 设置PID的目标值
返回值：无
实例：amcl_linear_ahead(0.1,0.25,1,1)
曾经出现的问题：加速度过大 导致机器人还没到达坐标点速度接近于0 进入死循环 
解决办法 ：Acceletare = Acceletare*0.9
*/
void linear_ahead_random(float V0,float V1,double Distance,char flag,char state,float pid_value)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   ros::Publisher send_pose = n1.advertise<geometry_msgs::Twist>("send_pose",2);//改为1

   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   ros::Rate rate(10.0);
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 0;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float All_adjust    = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   float current_yaw = 0;
   int flag_while = 0;
   float dirction_flag = 0;
   float speed_inc     = 0;
   while((dirction_flag>=0)&&ros::ok())
   {  
        ros::Time begin = ros::Time::now();
        try{
        //      listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        //      listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
              listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
              listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
          }
        catch(tf::TransformException& ex)
         {
           ROS_ERROR("nimei Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
           ros::Duration(1.0).sleep();
           continue;
         }
        q_linear.x() = transform_amcl_pose.getRotation().getX();
        q_linear.y() = transform_amcl_pose.getRotation().getY();
        q_linear.z() = transform_amcl_pose.getRotation().getZ();
        q_linear.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
        current_yaw = euler_linear[2]*57.29578;
        pid.Kp=0.5;//0.16 度直线参数
        pid.Ki=0.3;
        pid.Kd=0.08;
        adjust_amcl_Y       = PID_realize(pid_value,transform_amcl_pose.getOrigin().y());
        All_adjust = adjust_amcl_Y;  
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
       // Selection_action_mode = 0;//仅测试
       cout<<"OOOKKKKKSelection_action_mode = "<<Selection_action_mode<<endl;
        if(Selection_action_mode == 0){ //不存在障碍物
             All_adjust = All_adjust;
             Cur_speed  = get_run_speed(Cur_Distance,Distance,Cur_speed,V1,speed_inc);
             speed_inc = 0;
            speed_pre = Cur_speed;
          }
        else if(Selection_action_mode == 1) //表示离障碍物较远
        {
             ros::Time end = ros::Time::now();
             ros::Duration end_be = end - begin;
             if(Cur_speed>speed_pre)
               Cur_speed = speed_pre;//全局变量 
             float Acc = (Cur_speed*Cur_speed - 0.2*0.2)/(2.0*2);
            // cout<<"Cur_speed = "<<Cur_speed<<"  Acc1 = "<<Acc<<endl;
             Cur_speed = Cur_speed - Acc*end_be.toSec();//end_be.toSec() = 0.1
            // if(Cur_speed<0.05)
            // Cur_speed = 0.05;
            if(Cur_speed<0.2)
             Cur_speed = 0.2;
            speed_pre = Cur_speed;
            //All_adjust = All_adjust*10;
            //speed_inc = V0/2;
           //  cout<<" Cur_speed11 = "<< Cur_speed<<" min_lidar_data = "<<min_lidar_data<<"  inc_v = "<<(Cur_speed*Cur_speed - 0)/(2.0*min_lidar_data)*end_be.toSec()<<"  end_be.toSec() ="<<end_be.toSec()<<endl;
        }
        else{ //离障碍物很近 需要及停
              Buffer_emergency_stop(Cur_speed,5);
              Cur_speed  =  0;
              All_adjust =  0;
              speed_inc  =  V0;
             // Send_stop(); //完全停止后 在
             // ros::Duration(3).sleep(); 
              cout<<"state = 2 停止 = "<<Selection_action_mode<<"Speed_change_of_obstacle = "<<Speed_change_of_obstacle<<endl;
        }
        if (abs(Cur_speed)>max(abs(V0),abs(V1)))
        {
          Cur_speed = max(abs(V0),abs(V1));
          cout<<"OOOOOOOKKKKKKKKKKCur_speed = "<<Cur_speed<<endl;
        }
        int arc_in_flag = 1;
        if((abs(current_yaw)>4)&&(abs(transform_amcl_pose.getOrigin().y())<0.1))//0.04
          {
            if(((transform_amcl_pose.getOrigin().y()>0)&&(current_yaw<0))||((transform_amcl_pose.getOrigin().y()<0)&&(current_yaw>0)))
              {
                 All_adjust = Cur_speed*sin(euler_linear[2]/2)*sin(euler_linear[2]/2)/transform_amcl_pose.getOrigin().y();
                // cout<<"current_yaw = "<<current_yaw<<" euler_linear[2] = "<<euler_linear[2]<<"transform_amcl_pose.getOrigin().y() = "<<transform_amcl_pose.getOrigin().y()<<"All_adjust = "<<All_adjust<<endl;
                 arc_in_flag = 2;
              }
          }
        if (All_adjust<=-0.085) All_adjust = -0.085;//0.019
        if (All_adjust>0.085)  All_adjust = 0.085;
        //避障时可以在此处加入全局变量 判断发原来的速度还是发停止命令
       //不使用避障时屏蔽下行代码
        //Speed_change_of_obstacle = 0;//仅测试
        if(Speed_change_of_obstacle == 0){
           All_adjust = All_adjust;
        }
        else{
            if(Selection_action_mode == 1)
            Cur_speed = 0.2;
            cout<<"  Cur_speed = "<<Cur_speed<<" Speed_change_of_obstacle"<<Speed_change_of_obstacle<<endl;
            All_adjust = Speed_change_of_obstacle;
        }
        oFile_init<<"mode "<<Selection_action_mode<<" Cur_speed "<<Cur_speed<<" All_adjust "<<All_adjust<<" arc_in_flag "<<arc_in_flag<<" Speed_ob "<<Speed_change_of_obstacle<<" x "<<transform_amcl_pose.getOrigin().x()<<" y "<<transform_amcl_pose.getOrigin().y()<<" current_yaw "<<current_yaw<<endl;
        cout<<"  All_adjust = "<<All_adjust<<"  Cur_speed = "<<Cur_speed<<endl;
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = euler_linear[2];
        twist.angular.x = transform_amcl_pose.getOrigin().x();
        twist.angular.y = transform_amcl_pose.getOrigin().y();//
        twist.angular.z =All_adjust;//All_adjust
        send_vel1.publish(twist);
        send_pose.publish(twist);
       // Cur_speed  = V0;//暂时先这样设置 还需要配合其他(例：加速运动) 做出修改
        cout<<"  X = "<<transform_amcl_pose.getOrigin().x()<<" Y = "<<transform_amcl_pose.getOrigin().y()<<endl;
        //Cur_Distance = transform_linear.getOrigin().x();//里程坐标
        Cur_Distance = transform_amcl_pose.getOrigin().x();//amcl定位坐标
     //   cout << "amcl_x = "<<transform_amcl_pose.getOrigin().x() << " amcl_y = "<<transform_amcl_pose.getOrigin().y() << "Cur_speed ="<<Cur_speed<<endl ;
        if(flag ==0) dirction_flag = Distance - Cur_Distance;
        else  dirction_flag = Cur_Distance - Distance; 
        cout<<"dirction_flag = "<<dirction_flag<<" Cur_Distance = "<<Cur_Distance<<endl;
      //  oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<< " "<<"A"<<" "<<euler_linear[2]*57.29578<<" "<< "O_X"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<"g_odom_x"<<" "<<g_odom_x<< " "<<"g_odom_y"<<" "<<g_odom_y<<endl;
      //  rate.sleep();
   }
    //oFile_init<< "section ahead liner "<< " "<<endl;
}
void Arrival_Table()
{
   Arri_Table_flag = 1;
   Send_stop(); 
   Send_stop(); 
   cout<<"Arrival_TableOOOOOOOOOOOOOOOOOo"<<endl;
   delay_ms(5000);
 //  while(1);
   cout<<"KKKKKKKKKKKKKKKKKKK"<<endl;
   //ros::Duration(5).sleep();
   Arri_Table_flag = 0;
}
void Go_ahead()
{
    amcl_linear_ahead(0.1,0.25,1,1);//0.25
    cout<<"前进1"<<endl;
    amcl_linear_ahead(0.25,0.25,10.5,1);//10.5
     cout<<"前进2"<<endl;
    amcl_linear_ahead(0.25,0.2,11,1);//11.8
    cout<<"完成直线行驶"<<endl;
    
   // Arc_path_right(0.5,41.0,0.2,0.45*3.141*0.5,75);//80
    Arc_path_right_optimization(41.0,0.2,13,85,0);
    cout<<"完成弧线行驶"<<endl;
    Send_goahead(0.2);
    amcl_linear_Y_ahead(0.2,0.2,1.5,1,2.0);
    amcl_linear_Y_ahead(0.2,0.0,3,12.0,2.0);
    Send_stop(); 
    amcl_get_pose();  
    ros::Duration(1.0).sleep();
    Robot_Rotation_180(41.0,-0.1);
    Send_stop(); 
    ros::Duration(1.0).sleep();
   // Robot_Rotation(41.0,0.1,150);
    amcl_get_pose();
}
void Go_back()
{
    amcl_linear_Y_back(0.1,0.2,2,1);// amcl_linear_Y_ahead(0.1,0.2,2,1)
    //amcl_linear_Y_ahead(0.2,0.2,1,1);
    Arc_path_right_back_optimization(1,41.0,0.2,0,85);
   // Arc_path_right_back(1.5,41.0,-0.1,0.45*3.141*0.5,8);//
    //amcl_linear_back(-0.1,-0.1,10,0);//10.5
   // amcl_linear_back(-0.2,-0.25,10.5,0);//10.5
   // amcl_linear_back(0.2,0.25,10,1);
    Send_goahead(0.2);
    amcl_linear_back(0.2,0.25,10,1);
    cout<<"回到1"<<endl;
    amcl_linear_back(0.25,0.25,1.5,1);
     cout<<"回到2"<<endl;
    amcl_linear_back(0.25,0.0,0,1);
     cout<<"回到原点"<<endl;
   // amcl_linear_back(-0.25,-0.25,1,0);
   
  //  amcl_linear_back(-0.25,-0.0,0,0);
   
    Send_stop();
    amcl_get_pose();  
}
void Go_ahead_V02()
{
   amcl_linear_ahead_2(0.07,0.2,2,0,0,0);//
   amcl_linear_ahead_2(0.2,0.2,11.5,0,0,0);//0.25
   Arc_path_right_optimization(51.0,0.2,13.5,83,0);
   Send_goahead(0.2);
   //cout<<"amcl_linear_Y_ahead"<<endl;
   cout<<"0.6-min_speed = "<<0.6-min_speed<<endl;
   amcl_linear_Y_ahead(0.2,0.05,2.5,1,2.0);
   Send_stop();
   Send_stop();
  // amcl_linear_ahead_2(0.6,0.05,5,0,2,0);
   ros::Duration(3.0).sleep();
   Robot_Rotation_180(41.0,-0.1);
   Send_stop(); 
   ros::Duration(3.0).sleep();
   // Robot_Rotation(41.0,0.1,150);
   amcl_get_pose();
   amcl_get_pose();
   Send_stop();
   Send_stop();
}
void Go_back_V02()
{
    amcl_linear_Y_back(0.07,0.2,2,1);// amcl_linear_Y_ahead(0.1,0.2,2,1)
    //amcl_linear_Y_ahead(0.2,0.2,1,1);
    Arc_path_right_back_optimization(1,51.0,0.2,0,85);
   // Arc_path_right_back(1.5,41.0,-0.1,0.45*3.141*0.5,8);//
    //amcl_linear_back(-0.1,-0.1,10,0);//10.5
   // amcl_linear_back(-0.2,-0.25,10.5,0);//10.5
   // amcl_linear_back(0.2,0.25,10,1);
    cout<<"圆弧结束"<<endl;
    Send_goahead(0.2);
    cout<<"回到0"<<endl;
    amcl_linear_back(0.2,0.2,10,1);
    cout<<"回到1"<<endl;
    amcl_linear_back(0.2,0.2,2.5,1);
     cout<<"回到2"<<endl;
    amcl_linear_back(0.2,0.0,0,1);
     cout<<"回到原点"<<endl;
   // amcl_linear_back(-0.25,-0.25,1,0);
   
  //  amcl_linear_back(-0.25,-0.0,0,0);
   
    Send_stop();
    amcl_get_pose(); 
}

void Go_ahead_V06()
{
   amcl_linear_ahead_2(0.07,0.6,2,0,0,0);//
   amcl_linear_ahead_2(0.6,0.6,11.5,0,0,0);//0.25
   Arc_path_right_optimization(51.0,0.6,13.5,83,0);
   Send_goahead(0.46);
   //cout<<"amcl_linear_Y_ahead"<<endl;
   cout<<"0.6-min_speed = "<<0.6-min_speed<<endl;
   amcl_linear_Y_ahead(0.46,0.05,2.5,1,2.0);
   Send_stop();
   Send_stop();
  // amcl_linear_ahead_2(0.6,0.05,5,0,2,0);
   ros::Duration(2.0).sleep();
   Robot_Rotation_180(41.0,-0.1);
   Send_stop(); 
   ros::Duration(1.0).sleep();
   // Robot_Rotation(41.0,0.1,150);
   amcl_get_pose();
   amcl_get_pose();
   Send_stop();
   Send_stop();
}
void Go_back_V06()
{
    amcl_linear_Y_back(0.07,0.3,2,1);// amcl_linear_Y_ahead(0.1,0.2,2,1)
    //amcl_linear_Y_ahead(0.2,0.2,1,1);
    Arc_path_right_back_optimization(1,51.0,0.3,0,85);
   // Arc_path_right_back(1.5,41.0,-0.1,0.45*3.141*0.5,8);//
    //amcl_linear_back(-0.1,-0.1,10,0);//10.5
   // amcl_linear_back(-0.2,-0.25,10.5,0);//10.5
   // amcl_linear_back(0.2,0.25,10,1);
    cout<<"圆弧结束"<<endl;
    Send_goahead(0.3);
    cout<<"回到0"<<endl;
    amcl_linear_back(0.3,0.6,10,1);
    cout<<"回到1"<<endl;
    amcl_linear_back(0.6,0.6,3.0,1);
     cout<<"回到2"<<endl;
    amcl_linear_back(0.6,0,-0.25,1);//-0.06
     cout<<"回到原点"<<endl;
   // amcl_linear_back(-0.25,-0.25,1,0);
   
  //  amcl_linear_back(-0.25,-0.0,0,0);
   
    Send_stop();
    amcl_get_pose(); 
}


void linear_back_Random(float _Set_Point_X)
{
    Send_stop();
    ros::Duration(2.0).sleep();
    Robot_Rotation_180_X(51.0,-0.1);
    Send_stop();
    float back_speed = 0;
   // _Set_Point_X = 10;取消上位机控制
    // if(_Set_Point_X>4)
    //  back_speed = 0.4;
    // else back_speed = 0.2;
    // amcl_linear_back_Random(0.07,back_speed,_Set_Point_X*4/5,1);
    // amcl_linear_back_Random(back_speed,back_speed,_Set_Point_X/5,1);
    // amcl_linear_back_Random(back_speed,0.07,-0.06,1);
    back_speed = 0.2;
    amcl_linear_back_Random_Avoidance02(0.2,back_speed,_Set_Point_X*4/5,1);
    amcl_linear_back_Random_Avoidance02(back_speed,back_speed,_Set_Point_X/5,1);
    amcl_linear_back_Random_Avoidance02(back_speed,0.13,-0.06,1);
    Send_stop();
}

void Random_Pose_run()
{
  ros::Rate loop_rate(10);
//  while((g_Set_Points.points.size()==0)&&(ros::ok()))
  {
    cout<<"Set_Point_X = "<<Set_Point_X<<endl;
    cout<<"等待上位机发送指令"<<endl;
    ros::spinOnce();
   loop_rate.sleep();
  }
  float Set_X = 0;
  float Set_Y = 0;
 // for(unsigned int i = 0; i < g_Set_Points.points.size(); i++)
  {
        //cout<<"目标坐标 1"<<g_Set_Points.points[i]<<endl;
        //Set_X  = g_Set_Points.points[i].x;//Set_Point_X
        //Set_Y = g_Set_Points.points[i].y;//Set_Point_Y
        Set_X  = 6;//Set_Point_X
        Set_Y  = 0;//Set_Point_Y
        float line_distance = 0;
        if(Set_Y==0)
        {
            line_distance = Set_X;
            Arc_flag = 0;
        }
        else if(abs(Set_Y)<1)
        {
            Arc_flag = 1;
            line_distance = Set_X - Set_Y;
        }
        else{
          Arc_flag = 1;
          line_distance = Set_X - 2.0;
        }     
        // linear_ahead_random(0.07,0.4,line_distance,0,Increase_speed,0);
        linear_ahead_random(0.2,0.2,line_distance,0,Increase_speed,0);        
        if(Arc_flag == 0)
        {
            Send_stop();
            ros::Duration(2.0).sleep();
        }
  }
  if(Arc_flag == 1)
  {
      Arc_path_right_optimization_avoidance(51.0,0.4,Set_X,83,0);
      Send_goahead(0.36);
      //cout<<"amcl_linear_Y_ahead"<<endl;
      cout<<"0.6-min_speed = "<<0.6-min_speed<<endl;
      amcl_linear_Y_ahead(0.4,0.05,2.5,1,2.0);
      Send_stop();
      Send_stop();
      ros::Duration(2.0).sleep();
      Robot_Rotation_180(41.0,-0.1);
      Send_stop(); 
      ros::Duration(1.0).sleep();
      // Robot_Rotation(41.0,0.1,150);
      amcl_get_pose();
      amcl_get_pose();
      Send_stop();
      Send_stop();
      Go_back_V06();
  }
  else{
        linear_back_Random(Set_X);
      }
}

void Go_ahead_Avoidance()
{
    //amcl_linear_ahead_avoidance(0.2,0.2,5,0,Increase_speed,0);
    amcl_linear_ahead_avoidance(0.07,0.4,1.5,0,Increase_speed,0);
    amcl_linear_ahead_avoidance(0.4,0.4,6,0,Uniform_speed,0);
    amcl_linear_ahead_avoidance(0.4,0.4,10,0,Decrease_speed,0);
    Arc_path_right_optimization_avoidance(51.0,0.4,13.5,83,0);
    Send_goahead(0.36);
   //cout<<"amcl_linear_Y_ahead"<<endl;
    cout<<"0.6-min_speed = "<<0.6-min_speed<<endl;
    amcl_linear_Y_ahead(0.4,0.05,2.5,1,2.0);
    Send_stop();
    Send_stop();
  // amcl_linear_ahead_2(0.6,0.05,5,0,2,0);
    ros::Duration(2.0).sleep();
    Robot_Rotation_180(41.0,-0.1);
    Send_stop(); 
    ros::Duration(1.0).sleep();
    // Robot_Rotation(41.0,0.1,150);
    amcl_get_pose();
    amcl_get_pose();
    Send_stop();
    Send_stop();
}

void Go_back_Avoidance()
{
    amcl_linear_Y_back(0.07,0.3,2,1);// amcl_linear_Y_ahead(0.1,0.2,2,1)
    //amcl_linear_Y_ahead(0.2,0.2,1,1);
    Arc_path_right_back_optimization(1,51.0,0.3,0,85);
   // Arc_path_right_back(1.5,41.0,-0.1,0.45*3.141*0.5,8);//
    //amcl_linear_back(-0.1,-0.1,10,0);//10.5
   // amcl_linear_back(-0.2,-0.25,10.5,0);//10.5
   // amcl_linear_back(0.2,0.25,10,1);
    cout<<"圆弧结束"<<endl;
    Send_goahead(0.2);
    cout<<"回到0"<<endl;
    Arc_flag=0;
    amcl_linear_back_Avoidance(0.2,0.3,0,1,Increase_speed,Selection_action_mode,Speed_change_of_obstacle);
    cout<<"回到1"<<endl;
    //amcl_linear_back_Avoidance(0.4,0.4,3.0,1,Uniform_speed,Selection_action_mode,Speed_change_of_obstacle);
    // cout<<"回到2"<<endl;
    //amcl_linear_back_Avoidance(0.4,0.0,-0.05,1,Decrease_speed,Selection_action_mode,Speed_change_of_obstacle);
     cout<<"回到原点"<<endl;
   // amcl_linear_back(-0.25,-0.25,1,0);
   
  //  amcl_linear_back(-0.25,-0.0,0,0);
   
    Send_stop();
    amcl_get_pose(); 
}
void rectangle_run()
{
  cout<<"直线X 0  开始"<<endl;
  amcl_linear_ahead_2(0.1,0.2,1,0,0,0);//0.25
  cout<<"直线X 0  结束————圆弧外侧速度不变的弧线 0 运动  前进"<<endl;
  Arc_path_right_optimization(51.0,0.2,2.5,85,0);
  cout<<"直线Y 0  前进"<<endl;
  amcl_linear_Y_ahead(0.2,0.2,1.2,1,2.0);
  cout<<"圆弧外侧速度不变的弧线 0 运动  结束  到达桌子位置"<<endl;
  Arrival_Table();
  cout<<"圆弧外侧速度不变的弧线 1 运动  前进"<<endl;
  Arc_path_right_optimization(51.0,0.2,2.9,176,1);
 // cout<<"直线X 1  前进"<<endl;
  amcl_linear_ahead_2(0.2,0.2,-1,1,1,3.2);//0.25
 // cout<<"直线运行结束"<<endl;
 cout<<"圆弧外侧速度不变的弧线 2 运动  前进"<<endl;
  Arc_path_right_optimization(51.0,0.2,-2.3,-92,2);
  cout<<"圆弧外侧速度不变的弧线 3 运动  前进"<<endl;
  Arc_path_right_optimization(51.0,0.2,0.05,-1.5,3);
   cout<<"圆弧外侧速度不变的弧线 3 运动  结束  开始直线运动"<<endl;
  amcl_linear_ahead_2(0.2,0.2,0.0,0,1,0);//0.25

 // amcl_linear_ahead_2(0.2,0.2,4,1,1);
 // amcl_linear_ahead_2(0.2,0,5,1,2);
 // Send_stop(); 
}

void get_odom_pose_Callback(const riki_msgs::Velocities& pose) {
  g_odom_x = pose.linear_x;
  g_odom_y = pose.linear_y;
 // cout<<"g_odom_x = "<<g_odom_x<<" "<<"g_odom_y = "<<g_odom_y<<endl;
}

//void Avoidance_Callback(const riki_msgs::Velocities& pose)      custom_msg_topic::custom_msg msg;
// ros::Rate loop_rate(10);
/*
  说明：
    Selection_action_mode = 0：表示没有障碍物 正常运行
    Selection_action_mode = 1：表示有障碍物 离的较远 只进行语音通报 不减速
    Selection_action_mode = 2：表示有障碍物 离的较近 进行语音通报和减速
*/
void Avoidance_info_Callback(const custom_msg_topic::custom_msg& Danger) 
{
   ros::NodeHandle Danger_n1;
   ros::Publisher seng_state = Danger_n1.advertise<geometry_msgs::Twist>("state_val",2);//改为1
   //Arri_Table_flag = 1;
   Selection_action_mode = Danger.levels_of_anger;
   //Selection_action_mode = 0;//不进行避碍 屏蔽此行进行正常避碍物
   Speed_change_of_obstacle = Danger.Speed_change_of_obstacle;
   cout<<"Speed_change_of_obstacle = "<<Speed_change_of_obstacle<<endl;

  // cout<<"Speed_change_of_obstacle = "<<Speed_change_of_obstacle<<endl;
 //  for(int i = 0;i<10;i++)
 //  {
  //    Avoidance_Classification_group[i] = Danger.Avoidance_Classification_group[i];
     // cout<<Avoidance_Classification_group[i];
 //  }
  // cout<<endl;
   min_lidar_data        = Danger.min_lidar_data;
  cout<<" min_lidar_data = "<<min_lidar_data<<endl;
   if ((Selection_action_mode ==1)||(Selection_action_mode ==2))
      usart_send_data = 1;
   else if(Arri_Table_flag == 1) 
       usart_send_data = 2;
   else 
      usart_send_data = 0;
  //  cout<<"usart_send_data = "<<usart_send_data<<endl;
   geometry_msgs::Twist twist;
   twist.linear.x =usart_send_data; twist.linear.y = 0; twist.linear.z = 0;
   twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
   seng_state.publish(twist);
  // cout<<"Selection_action_mode = "<<usart_send_data<<" min_lidar_data = "<<min_lidar_data<<endl;
   //sleep(0.1);
}

void Set_Pose_Callback(const sensor_msgs::PointCloud& Points) 
{
  cout<<"size = "<<Points.points.size()<<endl;
  int pint_size    = Points.points.size();
  int passPose_num = Points.points[0].x;
  int tarPose_num  = Points.points[0].y;
  g_Set_Points.points.resize(tarPose_num);
  for(unsigned int i = 0; i < Points.points.size(); i++)
  {
     cout<<"坐标 "<<Points.points[i]<<endl;
  }
  for(unsigned int i = 0; i < tarPose_num; i++)
  {
     //cout<<"目标坐标 "<<Points.points[passPose_num+1+i]<<endl;
     g_Set_Points.points[i] = Points.points[passPose_num+1+i];
  }
  for(unsigned int i = 0; i < g_Set_Points.points.size(); i++)
  {
     cout<<"目标坐标 "<<g_Set_Points.points[i]<<endl;
  }
  if(pint_size>0)
  {
      Set_Point_X = Points.points[pint_size-1].x;
      Set_Point_Y = Points.points[pint_size-1].y;
  }

  //g_Set_Points = Points;
}
void Write_json()
{
    Json::Value root;
    // 组装json内容
    root["occupation"]  = "paladin";
    root["camp"]        = "alliance";
    root["role_id"]     = 1;
 
    // 将json内容（缩进格式）输出到文件
    Json::StyledWriter writer;
    ofstream os;
    os.open("test.json");
    os << writer.write(root);
}
void Read_json()
{
      Json::Value root;
      Json::Reader reader;
      //std::ifstream ifs("~/mini_catkin_ws/src/mini_robot_control/src/example1.json");//open file example.json
      // std::ifstream ifs("amcl_linear_x_ahead.cpp",std::ifstream::in);//open file example.json
      std::ifstream ifs("example.json");
      if(ifs.is_open())
     {
       cout<<"OKKKK"<<endl;
       std::cout<<"file is already open"<<endl;
      }

      if(ifs==NULL)
      {
        cout<<"NULL1"<<endl;
      }
     
      if(!reader.parse(ifs, root)){
        // fail to parse
        cout<<"失败"<<endl;
      }
      else{
        // success
        cout<<"解析成功"<<endl;
        Json::Value robot_Pose = root["content"];
        cout<<"targetPos ="<<robot_Pose["targetPos"]["x"]<<endl;
        cout<<"passPos ="<<robot_Pose["passPos"][1]["x"]<<endl;
        Json::Value obj2 = root["arrayobj2"];
        for(int i = 0;i < obj2.size();i++)
        {
          std::cout<<obj2[i]["name"].asString()<<endl;
          std::cout<<obj2[i]["age"].asInt()<<endl;
          std::cout<<obj2[i]["weight"].asDouble()<<endl;
        }

        std::cout<<root["encoding"].asString()<<endl;
        std::cout<<root["indent"]["length"].asInt()<<endl;
        std::cout<<root["filename"].asString()<<endl;
        std::cout<<root["string"].asString()<<endl;
        std::cout<<root["filesize"].asInt()<<endl;
        std::cout<<root["inttype"].asInt()<<endl;
        std::cout<<root["floattype"].asFloat()<<endl;
        std::cout<<root["doubletype"].asDouble()<<endl;
      }
    
}

int main(int argc, char **argv)
{ 
    //ofstream oFile;
	//oFile.open("test.csv",ios::out|ios::trunc);
   // for(int i = 0;i<10;i++)
   //      oFile<< "X "<< " "<<i<<" "<<"Y "<<" "<<i*i<<endl;
	//oFile.close();
    
 
    ros::init(argc,argv,"example");
    ros::NodeHandle n;
    ros::Publisher send_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",2);//改为1
    ros::Publisher seng_state = n.advertise<geometry_msgs::Twist>("state_val",2);//改为1
    ros::Subscriber sub = n.subscribe("get_odom_pose", 1, get_odom_pose_Callback);
    ros::Subscriber get_scan = n.subscribe("Avoidance_info", 1000, Avoidance_info_Callback);
    ros::Subscriber Set_Pose = n.subscribe("set_robot_pose", 1000, Set_Pose_Callback);
    tf::TransformListener listener_amcl_pose;
    tf::StampedTransform transform_amcl_pose;
    geometry_msgs::Twist twist;
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    send_vel.publish(twist);
    ros::Rate loop_rate(10);
    Eigen::Quaterniond q;
    tf::TransformListener listener;
    cout<<"System begin \n";
    PID_init();
    int           count=0;
    float angle_adjust = 0;
    float Angle_Arc    = 0;
    oFile_init.open("mini_pid_V.csv",ios::out|ios::trunc);
    tf::StampedTransform transform; 
    Eigen::Quaterniond q_Arc_path_test;
  //  Json::Value root;
  //  Json::Reader reader;
    //Read_json();
    //Write_json();
   // Arc_path_right(1.0,41.0,0.2,1.0,85);
  //  Arc_path_right_optimization(1,41.0,0.2,0.8,85);//(1,41.0,0.2,1.2,85)
   // Send_stop();
    //Go_ahead();
    cout<<"start————action"<<endl;
   // Arrival_Table();
   // for(int i = 0;i<3;i++)
  //  {
 //    rectangle_run();
  //  }
 // Go_ahead_V06();
  // Go_back_V06();

      Random_Pose_run();

   // Go_ahead_Avoidance();
   // Send_stop();
    //Go_back_Avoidance();
   
   //  cout<<"匀速"<<endl;
  // cout<<"直线X 0  开始"<<endl;
  // amcl_linear_ahead_2(0.1,0.2,0.8,0,0,0);//0.25
  // cout<<"直线X 0  结束————圆弧外侧速度不变的弧线 0 运动  前进"<<endl;
  // Arc_path_right_optimization(51.0,0.2,3,87,0);
   // Send_stop();
   // Send_stop();
     oFile_init.close();
    while(ros::ok())
    {
        cout<<" 退出直线运行 Speed_change_of_obstacle = "<<Speed_change_of_obstacle<<endl;
         //if(g_Set_Points.points.size()>0)
          // cout<<" g_Set_Points = "<<g_Set_Points.points[0]<<endl;
        amcl_get_pose(); 
       // run_get_pose();
        Send_stop();
      //auto_ration();
        
 /*
 
 
        amcl_linear_ahead_2(0.1,0.2,0.8,0,0,0);//0.25
        cout<<"Angle_Arc = "<<Angle_Arc<<endl;
        while(Angle_Arc<89) 
        {
          listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
          listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
          q_Arc_path_test.x() = transform_amcl_pose.getRotation().getX();
          q_Arc_path_test.y() = transform_amcl_pose.getRotation().getY();
          q_Arc_path_test.z() = transform_amcl_pose.getRotation().getZ();
          q_Arc_path_test.w() = transform_amcl_pose.getRotation().getW();
          Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
          cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
          Angle_Arc = euler_linear[2]*57.29578;
          cout<<"Angle_Arc = "<<Angle_Arc<<endl;
          auto_ration();
        }
        Send_stop();
        Send_stop();
        Send_stop();
        Send_stop();
    */
      //  while(ros::ok())
      //  cout<<"OOOOOOOOOOOKKKKKK"<<endl;
       // amcl_linear_ahead_2(0.2,0.2,-0.5,1,1,3.2);//0.25
        //auto_ration();
 //        cout<<"OKKK"<<endl;
       // print_amcl_linear_x_ahead();
       // amcl_linear_ahead(0.1,0.25,1,1);
       // Arc_path_right(0.707*0.75,41.0,0.2,0.45*3.141*0.5,85);
       // run_get_pose();
       // amcl_get_pose();
        ros::spinOnce();
        loop_rate.sleep();
    }
    oFile_init.close();
    return 0;
}
