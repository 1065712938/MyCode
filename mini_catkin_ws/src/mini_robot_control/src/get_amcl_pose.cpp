#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>
#include <mini_robot_control/Velocities.h>
#include <fstream>
#include <string>
using namespace std;
ofstream oFile_init;

double g_vel_x = 0.0;
double g_vel_y = 0.0;

double g_vel_dt = 0.0;
double g_imu_dt = 0.0;
double g_imu_z = 0.0;

ros::Time g_last_loop_time(0.0);
ros::Time g_last_vel_time(0.0);
ros::Time g_last_imu_time(0.0);
/*


void velCallback(const riki_msgs::Velocities& vel){
  //callback every time the robot's linear velocity is received
  ros::Time current_time = ros::Time::now();

  g_vel_x = vel.linear_x;
  g_vel_y = vel.linear_y;

  g_vel_dt = (current_time - g_last_vel_time).toSec();
  g_last_vel_time = current_time;
}

void IMUCallback( const sensor_msgs::Imu& imu){
  //callback every time the robot's angular velocity is received
  ros::Time current_time = ros::Time::now();
  //this block is to filter out imu noise
  if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
  {
    g_imu_z = 0.00;
  }else{
    g_imu_z = imu.angular_velocity.z;
  }
  g_imu_dt = (current_time - g_last_imu_time).toSec();
  g_last_imu_time = current_time;
}

int main(int argc, char** argv){
  double angular_scale, linear_scale;
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("raw_vel", 50, velCallback);
  ros::Subscriber imu_sub = n.subscribe("imu/data", 50, IMUCallback);

  nh_private_.getParam("angular_scale", angular_scale);
  nh_private_.getParam("linear_scale", linear_scale);

  double rate = 10.0;
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    ros::Ti——me current_time = ros::Time::now();

    //linear velocity is the linear velocity published from the Teensy board in x axis
    double linear_velocity_x = g_vel_x;

    //linear velocity is the linear velocity published from the Teensy board in y axis
    double linear_velocity_y = g_vel_y;

    //angular velocity is the rotation in Z from imu_filter_madgwick's output
    double angular_velocity = g_imu_z;

    //calculate angular displacement  θ = ω * t
    double delta_theta = angular_velocity * g_imu_dt * angular_scale; //radians
    double delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) * g_vel_dt * linear_scale; //m
    double delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) * g_vel_dt * linear_scale; //m

    //calculate current position of the robot
    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;
    g_last_loop_time = current_time;
    r.sleep();
  }
}


*/

void velCallback(const riki_msgs::Velocities& vel) {
  //callback every time the robot's linear velocity is received
  cout<<"ssssssss"<<endl;
  ros::Time current_time = ros::Time::now();
  g_vel_x = vel.linear_x;
  g_vel_y = vel.linear_y;
  g_vel_dt = (current_time - g_last_vel_time).toSec();
  g_last_vel_time = current_time;
        g_vel_x  +=g_vel_x;
     
      cout<<"g_vel_x = "<<g_vel_x<<endl;
}

void printHello()
{
  cout<<"Hello SLAM trajectory_plannin"<<endl;
}
void get_amcl_pose()
{
    tf::TransformListener listener_amcl_pose;
    tf::StampedTransform transform_amcl_pose;
    Eigen::Quaterniond q_amcl;
   // ros::Time begin = ros::Time::now();
    listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
    listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
    q_amcl.x() = transform_amcl_pose.getRotation().getX();
    q_amcl.y() = transform_amcl_pose.getRotation().getY();
    q_amcl.z() = transform_amcl_pose.getRotation().getZ();
    q_amcl.w() = transform_amcl_pose.getRotation().getW();
    Eigen::Vector3d euler_linear = q_amcl.toRotationMatrix().eulerAngles(0, 1, 2);
    cout << "  A = "<<euler_linear[2]*57.29578<<endl;
    ros::spinOnce();
    oFile_init<< "X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"Y"<<" "<<transform_amcl_pose.getOrigin().y()<<" "<<"A"<<" "<<euler_linear[2]*57.29578<<endl;
    cout << "  W_x = "<<transform_amcl_pose.getOrigin().x() << "  W_y = "<<transform_amcl_pose.getOrigin().y()<< endl ;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"get_amcl_pose");
  ros::NodeHandle ngap1;
   ros::Rate loop_rate_amcl(10);
  printHello();
 // get_amcl_pose();
  ros::Subscriber sub1 = ngap1.subscribe("raw_vel", 10, velCallback);
  ros::spin();
   oFile_init.open("get_amcl_pose.csv",ios::out|ios::trunc);
  
    while(ros::ok())
    {
      get_amcl_pose();
  //
       ros::spinOnce();

        loop_rate_amcl.sleep(); 
   }
  oFile_init.close();
  return 0;
}


