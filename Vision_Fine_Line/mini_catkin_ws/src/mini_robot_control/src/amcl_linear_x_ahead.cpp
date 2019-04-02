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
#include <fstream>
//#include "mini_robot_control/trajectory_plannin.h"
#include "mini_robot_control/amcl_linear_x_ahead.h"
#include <mini_robot_control/Velocities.h>

using namespace std;
void print_amcl_linear_x_ahead();
//void Arc_path_right_optimization(float R,float L,float speed,float Distance,float Set_Angle);

//void amcl_linear_ahead(float V0,float V1,double Distance,char flag);
void print_amcl_linear_x_ahead()
{
 cout<<"Hello SLAM amcl_linear_x_ahead"<<endl;
}

int main()
{
  //cout<<"Hello SLAM"<<endl;
 //printHello1();
 //PID_init();
 return 0;
}
