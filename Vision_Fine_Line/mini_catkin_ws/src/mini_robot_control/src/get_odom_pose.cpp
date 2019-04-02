#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <mini_robot_control/Odometry.h>
#include <mini_robot_control/Velocities.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h> //
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <math.h>
#include <fstream>
#include <string>
using namespace std;
ofstream oFile_get_odom_pose;

double g_vel_x = 0.0;
double g_vel_y = 0.0;

double g_vel_dt = 0.0;
double g_imu_dt = 0.0;
double g_imu_z = 0.0;

double W_x = 0;
double W_y = 0;

double inc_g_vel_x = 0;
int num_flag = 0;
int num_flag_amcl = 0;
ros::Time g_last_loop_time(0.0);
ros::Time g_last_vel_time(0.0);
ros::Time g_last_imu_time(0.0);

void velCallback(const riki_msgs::Velocities& vel) {
  ros::Time current_time = ros::Time::now();//实例化当前数据
  g_vel_x = vel.linear_x;
  g_vel_y = vel.linear_y;
  num_flag ++;//查看订阅的次数
  
  cout<<"num_flag = "<<num_flag<<"g_vel_x = "<<g_vel_x<<"g_vel_y = "<<endl;
  g_vel_dt = (current_time - g_last_vel_time).toSec();
  cout<<"current_time = "<<current_time<<" g_last_vel_time = "<<g_last_vel_time<<endl;
  g_last_vel_time = current_time;
}


void IMUCallback( const sensor_msgs::Imu& imu){
  ros::Time current_time = ros::Time::now();
  if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
  {
    g_imu_z = 0.00;
  }else{
    g_imu_z = imu.angular_velocity.z;
  }
  g_imu_dt = (current_time - g_last_imu_time).toSec();
  g_last_imu_time = current_time;
}


void AMCLCallback( const geometry_msgs::PoseWithCovarianceStamped & amcl_pose)
{
    W_x = amcl_pose.pose.pose.position.x;
    W_y = amcl_pose.pose.pose.position.y;
    num_flag_amcl ++;
    oFile_get_odom_pose<< "W_x"<< " "<<W_x<<" "<<"W_y"<<" "<<W_y<<" "<<"num_flag_amcl"<<" "<<num_flag_amcl<<endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_pose_tf");
  ros::NodeHandle n_get_odom_pose;
  oFile_get_odom_pose.open("2get_odom_pose.csv",ios::out|ios::trunc);
  tf::TransformListener  listener_linear;
  //tf::StampedTransform  transform_linear;
  oFile_get_odom_pose<< "test"<< " "<<endl;
  ros::Rate r(10);
  cout<<"get_odom_pose"<<endl;
  while(n_get_odom_pose.ok())
  {
    tf::StampedTransform  transform_linear;
   // ros::spinOnce();//进入订阅函数
    listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    //listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0));
    listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
    cout << "x11 = "<<transform_linear.getOrigin().x() << "  y = "<<transform_linear.getOrigin().y() << endl ;
    oFile_get_odom_pose<< "x_pos"<< " "<<transform_linear.getOrigin().x() <<" "<<"y_pos"<<" "<<transform_linear.getOrigin().y()<<endl;
    r.sleep();
  }
  oFile_get_odom_pose.close();
}

int main1(int argc, char** argv)
{
  ros::init(argc, argv, "odom_pose");
  ros::NodeHandle n_get_odom_pose;
  oFile_get_odom_pose.open("0get_odom_pose.csv",ios::out|ios::trunc);
  //ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n_get_odom_pose.subscribe("raw_vel", 50, velCallback);
  ros::Subscriber imu_sub = n_get_odom_pose.subscribe("imu/data", 50, IMUCallback);
  //ros::Subscriber amcl_pose_sub = n_get_odom_pose.subscribe("amcl_pose", 50, AMCLCallback);
  ros::Publisher get_odom_pose_pub = n_get_odom_pose.advertise< riki_msgs::Velocities>("get_odom_pose", 50);
  tf::TransformBroadcaster odom_broadcaster;
   double angular_scale = 0, linear_scale = 0;
  //nh_private_.getParam("angular_scale", angular_scale);
  //nh_private_.getParam("linear_scale", linear_scale);

  double rate = 10.0;
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;
  oFile_get_odom_pose<< "test"<< " "<<endl;
  ros::Rate r(rate);
  cout<<"get_odom_pose"<<endl;
  while(n_get_odom_pose.ok()){
    ros::spinOnce();//进入订阅函数
    ros::Time current_time = ros::Time::now();
    double linear_velocity_x = g_vel_x;

    double linear_velocity_y = g_vel_y;

    double angular_velocity = g_imu_z;
    if(g_imu_dt<0.09) g_imu_dt = 0.09;
    if(g_imu_dt>0.2)  g_imu_dt = 0.2;
    if(g_vel_dt<0.09) g_vel_dt = 0.09;
   if(g_vel_dt>0.2)  g_vel_dt = 0.2;

    //calculate angular displacement  θ = ω * t
    double delta_theta = angular_velocity * g_imu_dt*0.85 ; //radians *angular_scale
    double delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) * g_vel_dt*0.825; //m*linear_scale
    double delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) * g_vel_dt*0.825; //m*linear_scale
    //cout<<"angular_scale = "<<angular_scale<<endl;
    //calculate current position of the robot
    //inc_g_vel_x += linear_velocity_x ;
    inc_g_vel_x = linear_velocity_x * cos(theta);
    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;
  //  oFile_get_odom_pose<< "x_pos"<< " "<<x_pos<<" "<<"y_pos"<<" "<<y_pos<< " "<<" theta"<<" "<<theta<< " "<<" num_flag"<<" "<<num_flag<< " "<<" inc_g_vel_x"<<" "<<inc_g_vel_x<< " "<<" linear_velocity_x"<<" "<<linear_velocity_x<<endl;
    //cout<<"linear_velocity_x = "<<linear_velocity_x<<" linear_velocity_y = "<<linear_velocity_y<<endl;
    //cout<<"x_pos = "<<x_pos<<" y_pos = "<<y_pos<<" theta = "<<theta*57<<endl;
    g_last_loop_time = current_time;
    riki_msgs::Velocities get_odom_pose;
    get_odom_pose.linear_x = x_pos;
    get_odom_pose.linear_y = y_pos;
    get_odom_pose_pub.publish(get_odom_pose);
    cout<<"x_pos = "<<x_pos<<" y_pos = "<<y_pos<<" num_flag = "<<num_flag<<" inc_g_vel_x = "<<inc_g_vel_x<<" linear_velocity_x = "<<linear_velocity_x<<"g_vel_dt = "<<g_vel_dt<<"g_imu_dt = "<<g_imu_dt<<endl;
     // cout<<"W_x = "<<W_x<<" W_y = "<<W_y<<"  num_flag_amcl = "<<num_flag_amcl<<endl;
    oFile_get_odom_pose<< "x_pos"<< " "<<x_pos<<" "<<"y_pos"<<" "<<y_pos<<" "<<"g_vel_dt"<<" "<<g_vel_dt<<" "<<"g_imu_dt"<<" "<<g_imu_dt<<endl;
    r.sleep();
  }
  oFile_get_odom_pose.close();
}
