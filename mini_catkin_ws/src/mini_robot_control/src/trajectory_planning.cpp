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
#include "mini_robot_control/trajectory_plannin.h"
#include "custom_msg_topic/custom_msg.h"

using namespace std;
float flag_array[20]={0};
/*
flag_array[]标志位说明
flag_array[1] flag_array[4]  用于机器人自转 Robot_Rotation
flag_array[0]                用于圆弧 Arc_path 车体速度不变
flag_array[2]                用于圆弧 Arc_path_left 圆弧内侧车轮速度不变
flag_array[3]                用于圆弧 Arc_path_right 圆弧外侧车轮速度不变
flag_array[5]                用于amcl_linear_motion
flag_array[6] flag_array[7]  用于Track_pose
flag_array[8] flag_array[9]  flag_array[10] 用于 run_get_pose
flag_array[11]
*/
float g_odom_x = 0;
float g_odom_y = 0;
int Arc_flag   = 0;
float ranges[360]={0};
std::ofstream oFile_init;
void Send_stop();
void run_get_pose();
void amcl_get_pose();
void printHello()
{
  cout<<"Hello SLAM trajectory_plannin"<<endl;
}
//PID参考博客原文：https://blog.csdn.net/qq_28773183/article/details/79524766?utm_source=copy 
void PID_init();
float PID_realize(float Target_value,float Real_time_value);
void amcl_linear_back1(float V0,float V1,double Distance,char flag);
void Track_pose(float L,float K,float x,float y,float x1,float y1);
void Robot_Rotation(float L,float speed,float Set_Angle);
void amcl_linear_Y_back(float V0,float V1,double Distance,char flag);
void amcl_linear_back(float V0,float V1,double Distance,char flag);
void amcl_linear_back_Avoidance(float V0,float V1,double Distance,char flag,int mode ,float speed_obs);
void linear_motion(float V0,float V1,float Distance,char flag);
void linear_motion_goahead(float V0,float V1,float Distance,char flag,float Pid_set);
void Arc_path(float R,float L,float speed,float Distance);
void Arc_path_left(float R,float L,float speed,float Distance,float Set_Angle);
void Arc_path_right(float R,float L,float speed,float Distance,float Set_Angle);
void Arc_path_right_goahdead(float R,float L,float speed,float Distance,float Set_Angle);
void Arc_path_right_back(float R,float L,float speed,float Distance,float Set_Angle);
void Robot_Rotation_180(float L,float speed);
void Robot_Rotation_180_X(float L,float speed);
void Buffer_emergency_stop(float vel,int num);
float get_run_speed(double C_Pose,double G_Pose,float Cur_speed,float max_speed,float speed_inc);

void robot_line_Arc_path();
void Build_map();


/*
struct _pid{
                float SetSpeed;//定义设定值
                float ActualSpeed;//定义实际值
                float err;//定义偏差值
                float err_next;//定义下一个偏差值
                float err_last;//定义上一个偏差值
                float Kp,Ki,Kd;//定义比例、积分、微分系数
                float voltage;//定义电压值(控制执行器的变量)
                float integral;//定义积分值
           }pid;
*/
void PID_init()
{
        cout<<"PID_init begin \n";
        pid.SetSpeed=0.0;
        pid.ActualSpeed=0.0;
        pid.err=0.0;
        pid.err_next=0.0;
        pid.err_last=0.0;
        pid.voltage=0.0;
        pid.integral=0.0;
 //       pid.Kp=0.015;//角度直线参数
 //       pid.Ki=0.012;
 //       pid.Kd=0.005;
       pid.Kp=0.16;//W_Y参数
       pid.Ki=0.15;
       pid.Kd=0.08;
        cout<<"PID_init end \n";
}
float PID_realize(float Target_value,float Real_time_value)
{
    //cout<<"PID_realizellllllllllllllllll"<<endl;
    pid.SetSpeed=Target_value;
    pid.ActualSpeed = Real_time_value;
    pid.err=pid.SetSpeed-pid.ActualSpeed;
    float
    incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
    //pid.ActualSpeed+=incrementSpeed;
    pid.err_last=pid.err_next;
    pid.err_next=pid.err;
    return incrementSpeed;

    //return pid.ActualSpeed;
}





/*
 函数名：float get_run_speed(double C_Pose,double G_Pose)
 传入参数4个：
            C_Pose    : 当前坐标
            G_Pose    ：目标坐标
            Cur_speed : 当前速度值
            max_speed : 设置的最大速度
            speed_inc : 若停住后重新运行 赋值一个起始速度
返回值：Cur_speed
*/
float get_run_speed(double C_Pose,double G_Pose,float Cur_speed,float max_speed,float speed_inc)
{
    float Acceletare = 0;
    if((abs(G_Pose - C_Pose)<1)&&(Arc_flag==0))//减速 此处加上一个判断 是否需要 要跑弧线 则不减速 
    {
        Acceletare = (pow(0,2)-pow(Cur_speed,2))/(2*(1));
        if(Cur_speed<0.2) 
        Cur_speed  = Cur_speed + Acceletare*0.2+speed_inc;
        else Cur_speed  = Cur_speed + Acceletare*0.3+speed_inc;
        if(Cur_speed<0.05)
         Cur_speed = 0.05; 
    }
    else if(abs(G_Pose - C_Pose)<2)//匀速
    {
        Cur_speed  = Cur_speed*1.0;
    }
    else{                         //加速
        Acceletare = (pow(max_speed,2)-pow(Cur_speed,2))/(2*2.0);
        Cur_speed  = Cur_speed + Acceletare*0.1+speed_inc;
    }
    cout<<"Acceletare = "<<Acceletare<<"  Cur_speed = "<<Cur_speed<<endl;
    return Cur_speed;
}

/*
 函数名：void Send_stop()
 功能 ： 发送停止命令
返回值：无
*/
void Send_stop()
{
    cout<<"Send_stop"<<endl;
    ros::NodeHandle robot_stop;
    ros::Publisher send_stop = robot_stop.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    for(int i = 0;i<5;i++)
     {
        geometry_msgs::Twist twist;
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;//angle_adjust_linear
        send_stop.publish(twist);

     }
}

void Buffer_emergency_stop(float vel,int num)
{
    cout<<"Buffer_emergency_stop"<<endl;
    ros::NodeHandle Buffer_emergency_stop;
    ros::Publisher Send_goahead = Buffer_emergency_stop.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    float send_vel = 0;
    for(int i = 0;i<num;i++)
     {
        send_vel = send_vel - vel/num;
        if(send_vel<0) send_vel = 0;
        geometry_msgs::Twist twist;
        twist.linear.x = send_vel; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;//angle_adjust_linear
        Send_goahead.publish(twist);
     }
}

/*
 函数名：void Send_stop()
 功能 ： 发送停止命令
返回值：无
*/
void Send_goahead(float vel)
{
    cout<<"Send_goahead"<<endl;
    ros::NodeHandle robot_goahead;
    ros::Publisher Send_goahead = robot_goahead.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    for(int i = 0;i<3;i++)
     {
        geometry_msgs::Twist twist;
        twist.linear.x = vel; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;//angle_adjust_linear
        Send_goahead.publish(twist);
     }
}

/*
 函数名：void linear_motion
 功能 ： 根据三个非直线的点确定圆半径
 传入参数6个：
            3个坐标点
返回值：无

*/
/*
 函数名：void Verification_vel
 功能 ： 通过测量实际路程和时间 来验证速度
 传入参数2个：
        vel ： 传入的速度值 m/s
        time_s：延迟的时间 秒(s)
            
返回值：无

*/

void Verification_vel(float vel,int time_s)
{
    cout<<"Verification_vel"<<endl;
    ros::NodeHandle Verification_vel;
    ros::Publisher send_Verification_vel = Verification_vel.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    for(int i = 0;i<5;i++)
     {
        geometry_msgs::Twist twist;
        twist.linear.x = vel*1.095; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;//angle_adjust_linear
        send_Verification_vel.publish(twist);
     }
     ros::Duration(time_s).sleep();
     Send_stop();

}
float Calculate_cicular(float x1,float y1,float x2,float y2,float x3,float y3)
{
    //Calculate_cicular(0,0,1.4,2,0.7,1.5)
    float a, b, c, g, e, f,X,Y,R;
    e = 2 * (x2 - x1);
    f = 2 * (y2 - y1);
    g = x2*x2 - x1*x1 + y2*y2 - y1*y1;
    a = 2 * (x3 - x2);
    b = 2 * (y3 - y2);
    c = x3*x3 - x2*x2 + y3*y3 - y2*y2;
    X = (g*b - c*f) / (e*b - a*f);
    Y = (a*g - c*e) / (a*f - b*e);
    std::cout << "X = "<< X<<" Y = " << Y<< std::endl;
    R = sqrt((X-x1)*(X-x1)+(Y-y1)*(Y-y1));
    return R;
}

void run_get_pose()
{
    tf::TransformListener listener_pose;
    tf::StampedTransform transform_pose;
    Eigen::Quaterniond q;
    ros::Time begin = ros::Time::now();
    listener_pose.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    listener_pose.lookupTransform("/odom", "/base_link", ros::Time(0), transform_pose);
    q.x() = transform_pose.getRotation().getX();
    q.y() = transform_pose.getRotation().getY();
    q.z() = transform_pose.getRotation().getZ();
    q.w() = transform_pose.getRotation().getW();
    Eigen::Vector3d euler_linear = q.toRotationMatrix().eulerAngles(0, 1, 2);
    //cout << "  A = "<<euler_linear[2]*57.29578<<endl;
    ros::spinOnce();
    flag_array[8]  = transform_pose.getOrigin().x();
    flag_array[9]  = transform_pose.getOrigin().y();
    //cout << "  flag_array[8]11 = "<<flag_array[8] << "  flag_array[9] = "<<flag_array[9]<< endl ;
    cout << "  W_x1 = "<<transform_pose.getOrigin().x() << "  W_y = "<<transform_pose.getOrigin().y()<< endl ;
}

/*
 函数名：void amcl_get_pose
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:选择前进和后退 1 前进 0 后退
返回值：无
实例：amcl_linear_motion(0.1,0.25,1,1)
*/
void amcl_get_pose()
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
    cout << "  amcl_W_x = "<<transform_amcl_pose.getOrigin().x() << "  W_y = "<<transform_amcl_pose.getOrigin().y()<< "  A = "<<euler_linear[2]*57.29578<<endl ;
}

/*
 函数名：void amcl_linear_back1
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
void amcl_linear_back1(float V0,float V1,double Distance,char flag)
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
   if(abs(V1)<abs(V0)) 
      Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance))*0.8;
    else
       Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance));
   while((Cur_Distance>Distance)&&ros::ok()&&(flag_while<10))
   {
        cout<<"(abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
        cout << "x = "<<transform_linear.getOrigin().x() << endl ;
        q_linear.x() = transform_linear.getRotation().getX();
        q_linear.y() = transform_linear.getRotation().getY();
        q_linear.z() = transform_linear.getRotation().getZ();
        q_linear.w() = transform_linear.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);

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
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =All_adjust;//angle_adjust_linear+adjust_amcl_Yangle_adjust_linear -Cur_speed*0.05
        send_vel1.publish(twist);
        cout<<"Cur_speed = "<<Cur_speed<<endl;
        //Cur_Distance = transform_linear.getOrigin().x();//里程坐标
        Cur_Distance = transform_amcl_pose.getOrigin().x();//amcl定位坐标
        cout << "x = "<<transform_linear.getOrigin().x() << "  W_x = "<<transform_amcl_pose.getOrigin().x() << endl ;
        cout << "y = "<<transform_linear.getOrigin().y() << "  W_y = "<<transform_amcl_pose.getOrigin().y() << endl ;
        oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<< " "<<"A"<<" "<<euler_linear[2]*57.29578<<" "<< "O_X"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<endl;
   }
    oFile_init<< "section ahead liner "<< " "<<endl;
}
/*
 函数名：void Track_pose
 传入参数4个：
            R:圆弧的半径
            L：两个轮子间的距离 单位cm
            speed:为圆外侧速度 前进为正 后退为负 
            Distance:弧线端点的直线距离
返回值：无
例子  ：    Arc_path_right(0.707*0.75,41.0,0.2,0.45*3.141*0.5,85);
*/
void Track_pose(float L,float K,float x,float y,float x1,float y1)
{
    cout<<"圆弧外侧速度不变的弧线运动"<<endl;
    ros::Rate loop_rate_Arc_path(100);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    Eigen::Quaterniond q_Arc_path_test;
    float Angle_yaw = 0;
    float P = 0;
    float angle = 0;
    float speed = 0;
    float speed_wheel = 0;
    float pre_angle = 0;
    L = L/100.0;
    P = sqrt(pow((x1-x),2)+pow((y1-y),2));
    //double atan2(double y,double x) 
    flag_array[6] = flag_array[10];
    //flag_array[10] = atan(1)*57.29578;
    flag_array[10] = atan((y1-y)/(x1-x))*57.29578;
    
    if(flag_array[7] == 0)  flag_array[6] = flag_array[10];
    flag_array[7] = flag_array[7]+1;
    //if(flag_array[10] - flag_array[6]>5) flag_array[10] = flag_array[6]+5;
    //if(flag_array[10] - flag_array[6]<-5)  flag_array[10] = flag_array[6]-5;
   // cout<<"flag_array[6] = "<<flag_array[6]<<endl;
  //  cout<<" P = "<< P<<" flag_array[10] = "<<flag_array[10]<<endl;
    Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
    q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
    q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
    q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
    q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
    Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
   // cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
   // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
    //Angle_yaw = euler_linear[2]*57.29578;
    cout<<"angle_p= "<<flag_array[10]<<endl;
    Angle_yaw = euler_linear[2]*57.29578;
    cout << "Angle_yaw = "<< Angle_yaw << endl ;//得到的弧度值转化为角度值
    flag_array[10] = flag_array[10]+Angle_yaw;
    //angle    = angle-Angle_yaw;
    cout<<"P = "<<P<<endl;
    speed = K*P;
    speed_wheel = P*K*L*cos(flag_array[10]/57.29578)/P;//对应弧度值 
    cout<<"K*L*cos(flag_array[10]/57.29578) = "<<K*L*cos(flag_array[10]/57.29578)<<endl;
    cout<<"flag_array[10] = "<<flag_array[10]<<" speed = "<<speed<<" speed_wheel = "<<speed_wheel<<" cos(angle) = "<<cos(flag_array[10]/57.29578)<<endl;
    //右轮加 左轮减
    geometry_msgs::Twist twist;
    twist.linear.x =speed;twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =speed_wheel/L;//K
    send_vel1_Arc_path.publish(twist);
}
/*
 函数名：void Robot_Rotation
 功能 ：根据给定的角度 实现机器人原地自转（左右0-180）
 传入参数4个：
           
            L：两个轮子间的距离 单位cm
            speed:为圆自转速度 
                  speed大于0 逆时针 ，speed小于0 顺时针
            Set_Angle:所要旋转的角度 Set_Angle最好选择小于180度的值
返回值：无
说明 ：1. 当多次调用此函数前需要将flag_array[1]赋值为0;
      2. 返回的原始偏行角逆时针[0 180] 正上方为0 顺时针[0 -180] 加上180后变为0-360度
          ^  原始偏行角
          |0
          |
      90——————>-90
实例：Robot_Rotation(41.0,-0.05,100)
*/
void Robot_Rotation(float L,float speed,float Set_Angle)
{
    cout<<"自转运动"<<endl;
    ros::Rate loop_rate_Robot_Rotation(10);
    ros::NodeHandle Rotation;
    ros::Publisher send_vel1_Rotation = Rotation.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Rotation_test_linear;
    tf::StampedTransform transform_Rotation_test;
    Eigen::Quaterniond q_Rotation_test;
    L = L/100.0;
    float Angle_Rotation = 0;
    float Pre_Angle_Rotation = 0;
    char direction = 0;
    flag_array[1] = 0;
    Rotation_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    Rotation_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Rotation_test);
    q_Rotation_test.x() = transform_Rotation_test.getRotation().getX();
    q_Rotation_test.y() = transform_Rotation_test.getRotation().getY();
    q_Rotation_test.z() = transform_Rotation_test.getRotation().getZ();
    q_Rotation_test.w() = transform_Rotation_test.getRotation().getW();
    Eigen::Vector3d euler_linear = q_Rotation_test.toRotationMatrix().eulerAngles(0, 1, 2);
    cout << "x = "<<transform_Rotation_test.getOrigin().x() << endl ;
    cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
    Pre_Angle_Rotation = euler_linear[2]*57.29578+180;
    if(speed>=0)
       flag_array[4] = 1;
    else flag_array[4] = 0;
    while((flag_array[1]==0)&&ros::ok())
    {
        Rotation_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Rotation_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Rotation_test);
        q_Rotation_test.x() = transform_Rotation_test.getRotation().getX();
        q_Rotation_test.y() = transform_Rotation_test.getRotation().getY();
        q_Rotation_test.z() = transform_Rotation_test.getRotation().getZ();
        q_Rotation_test.w() = transform_Rotation_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Rotation_test.toRotationMatrix().eulerAngles(0, 1, 2);
        cout << "x = "<<transform_Rotation_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Rotation = euler_linear[2]*57.29578+180;//0-360度
        cout<<"Angle_Rotation = "<<Angle_Rotation<<"  Pre_Angle_Rotation = "<<Pre_Angle_Rotation<<endl;
    
     if (
        (flag_array[4]==1) //逆时针
        &&(Pre_Angle_Rotation+ Set_Angle>=360)
        &&( 
            ((Angle_Rotation <=(Set_Angle+Pre_Angle_Rotation-360))
                &&(Angle_Rotation >=0))
            ||
            ((Angle_Rotation <=360)&&(Angle_Rotation >=Pre_Angle_Rotation))
          )
        )
        {
            cout<<"逆时针"<<endl;
            flag_array[1] = 0;
            geometry_msgs::Twist twist;
            twist.linear.x =0.0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =speed/L;//K
            send_vel1_Rotation.publish(twist);
        }
       else if     
         (
           (flag_array[4]==0)
           &&((Pre_Angle_Rotation-Set_Angle)<0)
           &&(
               ((Angle_Rotation >=(360-(Set_Angle-Pre_Angle_Rotation)))
                    &&(Angle_Rotation <=360))
               ||
               ((Angle_Rotation <=Pre_Angle_Rotation)&&(Angle_Rotation >=0))
            )
         )
        {
             cout<<"顺时针"<<endl;
            flag_array[1] = 0;
            geometry_msgs::Twist twist;
            twist.linear.x =0.0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =speed/L;//K
            send_vel1_Rotation.publish(twist);
        } 
        else if(abs(Angle_Rotation-Pre_Angle_Rotation)<=Set_Angle)
        {
            cout<<"正常模式"<<endl;
            flag_array[1] = 0;
            geometry_msgs::Twist twist;
            twist.linear.x =0.0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =speed/L;//K
            send_vel1_Rotation.publish(twist);
        } 
        else
        {
            cout<<"flag_array[1] = "<<flag_array[1]<<endl;
             geometry_msgs::Twist twist;
            twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
            send_vel1_Rotation.publish(twist);
            flag_array[1] = 1;
        }
        loop_rate_Robot_Rotation.sleep();
    }
}
/*
 函数名：void amcl_linear_Y_back
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:选择前进和后退 1 前进 0 后退
返回值：无
实例：amcl_linear_Y_back(0.1,0.25,1,1)
曾经出现的问题：加速度过大 导致机器人还没到达坐标点速度接近于0 进入死循环 
解决办法 ：Acceletare = Acceletare*0.9
*/
void amcl_linear_Y_back(float V0,float V1,double Distance,char flag)
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
   if(abs(V1)<abs(V0)) 
      Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance))*0.8;
    else
       Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance));
    oFile_init<< "section amcl_linear_Y_ahead "<< " "<<endl;
   while((Cur_Distance>Distance)&&ros::ok()&&(flag_while<10))
   {
        cout<<"(abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
        cout << "x = "<<transform_linear.getOrigin().x() << endl ;
        q_linear.x() = transform_linear.getRotation().getX();
        q_linear.y() = transform_linear.getRotation().getY();
        q_linear.z() = transform_linear.getRotation().getZ();
        q_linear.w() = transform_linear.getRotation().getW();
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
       adjust_amcl_Y       = -PID_realize(13,transform_amcl_pose.getOrigin().x());
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
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =All_adjust;//All_adjust
        send_vel1.publish(twist);
        cout<<"Cur_speed = "<<Cur_speed<<endl;
        //Cur_Distance = transform_linear.getOrigin().x();//里程坐标
        Cur_Distance = transform_amcl_pose.getOrigin().y();//amcl定位坐标
        cout << "x = "<<transform_linear.getOrigin().x() << "  W_x = "<<transform_amcl_pose.getOrigin().x() << endl ;
        cout << "y = "<<transform_linear.getOrigin().y() << "  W_y = "<<transform_amcl_pose.getOrigin().y() << endl ;
        oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<< " "<<"A"<<" "<<euler_linear[2]*57.29578<<" "<< "O_X"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<"g_odom_x"<<" "<<g_odom_x<< " "<<"g_odom_y"<<" "<<g_odom_y<<endl;
   } 
}
/*
 函数名：void amcl_linear_back
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
void amcl_linear_back(float V0,float V1,double Distance,char flag)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   int flag_plus = 0;
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 255;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float All_adjust    = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   int flag_while = 0;
   float current_yaw = 0;
   listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
   listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
    pre_odom_distance = abs(transform_linear.getOrigin().x());
 //  if(abs(V1)<abs(V0)) 
 //     Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance))*0.8;
 //   else
  //     Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance));
 //   oFile_init<< "section back liner "<< " "<<endl;
 cout<<"Distance-0.1 = "<<Distance<<endl;
   while((Cur_Distance>Distance)&&ros::ok()&&(flag_while<10))
   {
   //     cout<<"(abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
    //    cout << "x = "<<transform_linear.getOrigin().x() << endl ;
        q_linear.x() = transform_amcl_pose.getRotation().getX();
        q_linear.y() = transform_amcl_pose.getRotation().getY();
        q_linear.z() = transform_amcl_pose.getRotation().getZ();
        q_linear.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
        current_yaw = euler_linear[2]*57.29578;
       // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
       // angle_adjust_linear = PID_realize(0,euler_linear[2]*57.29578);
       // adjust_amcl_Y       = PID_realize(0,transform_amcl_pose.getOrigin().y());
      //  pid.Kp=0.18;//角度直线参数
      //  pid.Ki=0.2;
       // pid.Kd=0.1;
       pid.Kp=0.65;//角度直线参数 0.16 0.5 0.6 0.77
       pid.Ki=0.7;//0.15 0.5 .7
       pid.Kd=0.16;//0.08
        adjust_amcl_Y       = -PID_realize(0,transform_amcl_pose.getOrigin().y());
        //All_adjust = 0.8*angle_adjust_linear + adjust_amcl_Y*1.2-Cur_speed*0.05;
        All_adjust = adjust_amcl_Y;
       // cout << "All_adjust = "<<All_adjust << endl;

         
        //if (angle_adjust_linear<=-0.01281)  angle_adjust_linear = -0.01281;
        //if (angle_adjust_linear>0.01281) angle_adjust_linear = 0.01281;
        
      //  cout << "angle_adjust_linear = "<<angle_adjust_linear << "  A = "<<euler_linear[2]*57.29578<<endl;
      //  cout <<"amcl.y = "<<transform_amcl_pose.getOrigin().y()<< "  adjust_amcl_Y = "<<adjust_amcl_Y<< endl;
        //cout<<"Cur_speed = "<<Cur_speed<<endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
        Cur_speed  = Cur_speed + Acceletare*(end_be.toSec());
        Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*abs(Cur_Distance-Distance));
       // cout<<"Cur_speed = "<<Cur_speed<<"Acceletare = "<<Acceletare<<endl;
        if (abs(Cur_speed)>max(abs(V0),abs(V1)))
            Cur_speed = max(abs(V0),abs(V1));
        if (abs(Cur_speed)<=0.05)
            {
               Cur_speed = 0.05;
            //   flag_while = flag_while+1;
            }
           
      //  cout<<"flag_while = "<<flag_while<<endl;
        if(flag == 1) //前进
           Cur_speed = abs(Cur_speed);
        else
           Cur_speed = -abs(Cur_speed);
        if((180-abs(current_yaw)>1)&&(abs(transform_amcl_pose.getOrigin().y())<0.04))//0.04 0.03
        {
            if(((transform_amcl_pose.getOrigin().y()>0)&&((current_yaw<-90)&&(current_yaw>-180)))||((transform_amcl_pose.getOrigin().y()<0)&&((current_yaw>90)&&(current_yaw<180))))
              {
                 All_adjust = -Cur_speed*sin((3.141592654-abs(euler_linear[2]))/2.0)*sin((3.141592654-abs(euler_linear[2]))/2.0)/transform_amcl_pose.getOrigin().y();
                 //if((All_adjust>-0.01)&&(All_adjust<0))
                 //All_adjust = -0.01;
                // if((All_adjust<0.01)&&(All_adjust>0))
                 //All_adjust = 0.01;
                 flag_plus++;
                 cout<<"spac_All_adjust = "<<All_adjust<<"euler_linear[2]/2 = "<<euler_linear[2]/2<<"  sin = "<<sin((3.141592654-abs(euler_linear[2]))/2.0)<<"sin(1.57) = "<<sin(1.57)<<"W_y = "<<transform_amcl_pose.getOrigin().y()<<"Cur_speed = "<<Cur_speed<<endl;
              }
        }

 
        if (All_adjust<=-0.05) All_adjust = -0.05;//0.025
        if (All_adjust>0.05)  All_adjust = 0.05;

        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =All_adjust;//All_adjust angle_adjust_linear+adjust_amcl_Yangle_adjust_linear -Cur_speed*0.05
        send_vel1.publish(twist);
       // cout<<"Cur_speed = "<<Cur_speed<<" All_adjust = "<<All_adjust<<endl;
        //Cur_Distance = transform_linear.getOrigin().x();
        Cur_Distance = transform_amcl_pose.getOrigin().x();
        cout << "W_x = "<<transform_amcl_pose.getOrigin().x() << "  W_y = "<<transform_amcl_pose.getOrigin().y() <<"flag_plus = "<<flag_plus<< endl ;
       // cout << "x = "<<transform_linear.getOrigin().x() << "  y = "<<transform_linear.getOrigin().y() << endl ;
       // oFile_init<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<endl;
        oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<<"  "<<"A"<<"  "<<euler_linear[2]*57.29578<<" "<< "All_adjust1"<<"  "<<All_adjust<<" "<<"flag_plus "<<flag_plus<<" "<< "OX"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<endl;
   }
    oFile_init<< "section back liner "<< " "<<endl;
}
int   Selection_action_mode = 0;
float Speed_change_of_obstacle = 0;

void Avoidance_Callback(const custom_msg_topic::custom_msg& Danger) 
{
       Selection_action_mode = Danger.levels_of_anger;
   //Selection_action_mode = 0;//不进行避碍 屏蔽此行进行正常避碍物
      Speed_change_of_obstacle = Danger.Speed_change_of_obstacle;
}

/*
 函数名：void amcl_linear_back_Avoidance
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
void amcl_linear_back_Avoidance(float V0,float V1,double Distance,char flag,int state,int mode,float speed_obs)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   ros::Publisher send_pose = n1.advertise<geometry_msgs::Twist>("send_pose",2);//改为1
   ros::Subscriber get_avoidance = n1.subscribe("Avoidance_info", 1000, Avoidance_Callback);

   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   tf::TransformListener listener_amcl_pose;
   tf::StampedTransform transform_amcl_pose;
   Eigen::Quaterniond q_linear;
   int flag_plus = 0;
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   double Cur_Distance = 255;
   float amcl_X = 0;
   float amcl_Y = 0;
   float adjust_amcl_Y = 0;
   float All_adjust    = 0;
   float pre_amcl_distance = 0;
   double pre_odom_distance = 0;
   int flag_while = 0;
   float current_yaw = 0;
   float speed_inc     = 0;
   float speed_pre       = 0;

   listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
   listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
    pre_odom_distance = abs(transform_linear.getOrigin().x());
 //  if(abs(V1)<abs(V0)) 
 //     Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance))*0.8;
 //   else
  //     Acceletare = (pow(V1,2)-pow(V0,2))/(2*(Distance - pre_odom_distance));
 //   oFile_init<< "section back liner "<< " "<<endl;
 cout<<"Distance-0.1 = "<<Distance<<endl;
   while((Cur_Distance>Distance)&&ros::ok()&&(flag_while<10))
   {
   //     cout<<"(abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        listener_amcl_pose.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_amcl_pose.lookupTransform("/map", "/base_link", ros::Time(0), transform_amcl_pose);
    //    cout << "x = "<<transform_linear.getOrigin().x() << endl ;
        q_linear.x() = transform_amcl_pose.getRotation().getX();
        q_linear.y() = transform_amcl_pose.getRotation().getY();
        q_linear.z() = transform_amcl_pose.getRotation().getZ();
        q_linear.w() = transform_amcl_pose.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
        current_yaw = euler_linear[2]*57.29578;
       // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
       // angle_adjust_linear = PID_realize(0,euler_linear[2]*57.29578);
       // adjust_amcl_Y       = PID_realize(0,transform_amcl_pose.getOrigin().y());
      //  pid.Kp=0.18;//角度直线参数
      //  pid.Ki=0.2;
       // pid.Kd=0.1;
       pid.Kp=0.65;//角度直线参数 0.16 0.5 0.6 0.77
       pid.Ki=0.7;//0.15 0.5 .7
       pid.Kd=0.16;//0.08
       adjust_amcl_Y       = -PID_realize(0,transform_amcl_pose.getOrigin().y());
        All_adjust = adjust_amcl_Y;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
        //Cur_speed  = Cur_speed + Acceletare*(end_be.toSec());
        //Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*abs(Cur_Distance-Distance));

           
        if(flag == 1) //前进
           Cur_speed = abs(Cur_speed);
        else
           Cur_speed = -abs(Cur_speed);
        if((180-abs(current_yaw)>1)&&(abs(transform_amcl_pose.getOrigin().y())<0.04))//0.04 0.03
        {
            if(((transform_amcl_pose.getOrigin().y()>0)&&((current_yaw<-90)&&(current_yaw>-180)))||((transform_amcl_pose.getOrigin().y()<0)&&((current_yaw>90)&&(current_yaw<180))))
              {
                 All_adjust = -Cur_speed*sin((3.141592654-abs(euler_linear[2]))/2.0)*sin((3.141592654-abs(euler_linear[2]))/2.0)/transform_amcl_pose.getOrigin().y();
                 //if((All_adjust>-0.01)&&(All_adjust<0))
                 //All_adjust = -0.01;
                // if((All_adjust<0.01)&&(All_adjust>0))
                 //All_adjust = 0.01;
                 flag_plus++;
                 //cout<<"spac_All_adjust = "<<All_adjust<<"euler_linear[2]/2 = "<<euler_linear[2]/2<<"  sin = "<<sin((3.141592654-abs(euler_linear[2]))/2.0)<<"sin(1.57) = "<<sin(1.57)<<"W_y = "<<transform_amcl_pose.getOrigin().y()<<"Cur_speed = "<<Cur_speed<<endl;
              }
        }
        if(Selection_action_mode == 0){ //不存在障碍物
             All_adjust = All_adjust;
             Cur_speed  = get_run_speed(Cur_Distance,Distance,Cur_speed,V1,speed_inc);
             /*
             if(state == 0) //0 表示加速状态
             {        
                Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
                Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*abs(Distance - Cur_Distance));
                cout<<"Acceletare = "<<Acceletare<<"  end_time = "<<end_be.toSec()<<endl;
             }
             else
              {
                //在匀速阶段要判断 Distance - Cur_Distance 值过大 考虑给一个固定值
                Acceletare = (pow(V1,2)-pow(Cur_speed,2))/(2*abs(Distance - Cur_Distance));
                Cur_speed  = Cur_speed + Acceletare*(end_be.toSec())+speed_inc;
              }
              */

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
              speed_inc  =  0.1;//V0
             // Send_stop(); //完全停止后 在
             // ros::Duration(3).sleep(); 
         //     cout<<"state = 2 停止 = "<<Selection_action_mode<<"Speed_change_of_obstacle = "<<Speed_change_of_obstacle<<endl;
        }

      if (abs(Cur_speed)>max(abs(V0),abs(V1)))
            Cur_speed = max(abs(V0),abs(V1));
        //if (abs(Cur_speed)<=0.05)
        //    {
        //       Cur_speed = 0.05;
        //    }
        //if (All_adjust<=-0.05) All_adjust = -0.05;//0.025
        //if (All_adjust>0.05)  All_adjust = 0.05;
    
        if (All_adjust<=-0.085) All_adjust = -0.085;//0.019
        if (All_adjust>0.085)  All_adjust = 0.085;

       if(Speed_change_of_obstacle == 0){
           All_adjust = All_adjust;
        }
        else{
            if(Selection_action_mode == 1)
            Cur_speed = 0.2;
        //    cout<<"  Cur_speed = "<<Cur_speed<<" Speed_change_of_obstacle"<<Speed_change_of_obstacle<<endl;
            All_adjust = Speed_change_of_obstacle;
        }
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y =1; 
        twist.linear.z = euler_linear[2];
        twist.angular.x = transform_amcl_pose.getOrigin().x();
        twist.angular.y = transform_amcl_pose.getOrigin().y(); 
        twist.angular.z =All_adjust;//All_adjust angle_adjust_linear+adjust_amcl_Yangle_adjust_linear -Cur_speed*0.05
        send_vel1.publish(twist);
        send_pose.publish(twist);
       // cout<<"Cur_speed = "<<Cur_speed<<" All_adjust = "<<All_adjust<<endl;
        //Cur_Distance = transform_linear.getOrigin().x();
        Cur_Distance = transform_amcl_pose.getOrigin().x();
        oFile_init<< "Selection_action_mode "<<Selection_action_mode<<" Speed_change_of_obstacle "<<Speed_change_of_obstacle<<" All_adjust "<<All_adjust<<" Cur_speed "<<Cur_speed<<" W_x "<<transform_amcl_pose.getOrigin().x()<<endl;
        cout<< "Selection_action_mode "<<Selection_action_mode<<" Speed_change_of_obstacle "<<Speed_change_of_obstacle<<" All_adjust "<<All_adjust<<" Cur_speed "<<Cur_speed<<" W_x "<<transform_amcl_pose.getOrigin().x()<<endl;
        cout << "W_x = "<<transform_amcl_pose.getOrigin().x() << "  W_y = "<<transform_amcl_pose.getOrigin().y() <<"flag_plus = "<<flag_plus<< endl ;
       // cout << "x = "<<transform_linear.getOrigin().x() << "  y = "<<transform_linear.getOrigin().y() << endl ;
       // oFile_init<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<endl;
        //oFile_init<< "W_X"<< " "<<transform_amcl_pose.getOrigin().x()<<" "<<"W_Y"<<" "<<transform_amcl_pose.getOrigin().y()<<"  "<<"A"<<"  "<<euler_linear[2]*57.29578<<" "<< "All_adjust1"<<"  "<<All_adjust<<" "<<"flag_plus "<<flag_plus<<" "<< "OX"<< " "<<transform_linear.getOrigin().x()<<" "<<"O_Y"<<" "<<transform_linear.getOrigin().y()<< " "<<endl;
   }
    oFile_init<< "section back liner "<< " "<<endl;
}
//void Go_ahead();
/*
 函数名：void linear_motion
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:选择前进和后退 1 前进 0 后退
返回值：无
实例： linear_motion(-0.08,-0.1,-1,0);
*/
void linear_motion(float V0,float V1,float Distance,char flag)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   Eigen::Quaterniond q_linear;
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   float Cur_Distance = 0;
   Acceletare = (pow(V1,2)-pow(V0,2))/(2*Distance);
   cout<<"abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
   while((abs(Cur_Distance-abs(Distance))>0.02)&&ros::ok())
   {
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        q_linear.x() = transform_linear.getRotation().getX();
        q_linear.y() = transform_linear.getRotation().getY();
        q_linear.z() = transform_linear.getRotation().getZ();
        q_linear.w() = transform_linear.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
       // cout << "x = "<<transform_linear.getOrigin().x() << endl ;
       // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        angle_adjust_linear=PID_realize(90.0,euler_linear[2]*57.29578);
        angle_adjust_linear +=angle_adjust_linear;
        if (angle_adjust_linear<=-0.0128)
        angle_adjust_linear = -0.0128;
        if (angle_adjust_linear>0.0128)
        angle_adjust_linear = 0.0128;
        cout << "angle_adjust_linear = "<<angle_adjust_linear << endl;
       // cout<<"Cur_speed = "<<Cur_speed<<endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
      //  cout<<"end_be.toSec() = "<<end_be.toSec()<<"Acceletare = "<<Acceletare<<endl;
        Cur_speed  = Cur_speed + Acceletare*(end_be.toSec());
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;                  //0.042 0.047 0.05
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angle_adjust_linear-Cur_speed*0.05;//angle_adjust_linear
        send_vel1.publish(twist);
        if(flag == 1) //前进
           Cur_speed = abs(Cur_speed);
        else
           Cur_speed = -abs(Cur_speed);
         Cur_Distance = Cur_Distance+abs(Cur_speed)*(end_be.toSec());
        
        //cout<<"Cur_Distance = "<<Cur_Distance<<" Distance = "<<Distance<<endl;
        //cout<<"abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
   }
     
}
/*
 函数名：void linear_motion_goahead
 传入参数4个：
            V0:初始速度
            V1：末速度
            Distance:加速减速距离 前进为正 后退为负
            flag:选择前进和后退 1 前进 0 后退
返回值：无
实例： linear_motion_goahead(-0.08,-0.1,-1,0);
*/
void linear_motion_goahead(float V0,float V1,float Distance,char flag,float Pid_set)
{
   ros::NodeHandle n1;
   ros::Publisher send_vel1 = n1.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
   tf::TransformListener listener_linear;
   tf::StampedTransform transform_linear;
   Eigen::Quaterniond q_linear;
   float angle_adjust_linear = 0;
   float Acceletare = 0; 
   float Cur_speed  = V0;
   double dur_time  = 0;
   float Cur_Distance = 0;
   Acceletare = (pow(V1,2)-pow(V0,2))/(2*Distance)*0.7;
   cout<<"abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
   while((Cur_Distance<abs(Distance))&&ros::ok())
   {
        ros::Time begin = ros::Time::now();
        listener_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_linear);
        q_linear.x() = transform_linear.getRotation().getX();
        q_linear.y() = transform_linear.getRotation().getY();
        q_linear.z() = transform_linear.getRotation().getZ();
        q_linear.w() = transform_linear.getRotation().getW();
        Eigen::Vector3d euler_linear = q_linear.toRotationMatrix().eulerAngles(0, 1, 2);
       // cout << "x = "<<transform_linear.getOrigin().x() << endl ;
       // cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        angle_adjust_linear=PID_realize(Pid_set,euler_linear[2]*57.29578);
        angle_adjust_linear +=angle_adjust_linear;
        if (angle_adjust_linear<=-0.0128)
        angle_adjust_linear = -0.0128;
        if (angle_adjust_linear>0.0128)
        angle_adjust_linear = 0.0128;
        cout << "angle_adjust_linear = "<<angle_adjust_linear << endl;
       // cout<<"Cur_speed = "<<Cur_speed<<endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin;
      //  cout<<"end_be.toSec() = "<<end_be.toSec()<<"Acceletare = "<<Acceletare<<endl;
        Cur_speed  = Cur_speed + Acceletare*(end_be.toSec());
        geometry_msgs::Twist twist;
        twist.linear.x = Cur_speed; twist.linear.y = 0; twist.linear.z = 0;                  //0.042 0.047 0.05
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =angle_adjust_linear-Cur_speed*0.05;// angle_adjust_linear-Cur_speed*0.05
        send_vel1.publish(twist);
        if(flag == 1) //前进
           Cur_speed = abs(Cur_speed);
        else
           Cur_speed = -abs(Cur_speed);
         Cur_Distance = Cur_Distance+abs(Cur_speed)*(end_be.toSec());
        
        //cout<<"Cur_Distance = "<<Cur_Distance<<" Distance = "<<Distance<<endl;
        //cout<<"abs(Cur_Distance-Distance) = "<<abs(Cur_Distance-Distance)<<endl;
   }
     
}


/*
 函数名：void Arc_path
 传入参数4个：
            R:圆弧的半径
            L：两个轮子间的距离 单位cm
            speed:车体速度 前进为正 后退为负
            Distance:弧线端点的直线距离
返回值：无
实例  ：Arc_path(1.0,41.0,0.1,3.141*0.75)
*/
void Arc_path(float R,float L,float speed,float Distance)
{
    ros::Rate loop_rate_Arc_path(10);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    float Rotate_1=0;
    float R_1=0;
    float Cur_Distance = 0;
    float Rotate = -((200/L)*R+1) /(1-(200/L)*R);
    Rotate=(Rotate - 1)/(Rotate + 1)*speed;
    cout<<"Rotate = "<<Rotate<<"  K = "<<L*speed/200.0/R<<endl;
    Rotate_1 = Rotate/0.59;
    R_1 = 0.41*(speed+Rotate_1+speed)/2/(2*Rotate_1);
    cout<<"Rotate_1 = "<<Rotate_1<<"  R_1 = "<<R_1<<endl;
    

    cout<<"S-Rotate = "<<speed-Rotate/0.59<<"  S+Rotate = "<<speed+Rotate/0.59<<endl;

    geometry_msgs::Twist twist;
	twist.linear.x =speed;twist.linear.y = 0; twist.linear.z = 0;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =Rotate/L;
    send_vel1_Arc_path.publish(twist);
    
    ros::spinOnce();
    
    cout<<"Distance = "<<Distance<<"flag_array[0] = "<<flag_array[0]<<endl;
    while((flag_array[0]==0)&&ros::ok())
    {
        if(abs(Cur_Distance-abs(Distance))>0.02)
        {
            flag_array[0] = 0;
            ros::Time begin = ros::Time::now();
            ros::Duration(0.1).sleep();
            ros::Time end = ros::Time::now();
            ros::Duration end_be = end - begin;
            Cur_Distance = Cur_Distance+abs(speed-Rotate*1.5)*(end_be.toSec());
            geometry_msgs::Twist twist;
            twist.linear.x =speed;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =Rotate*1.08;
            send_vel1_Arc_path.publish(twist);
        }
        else
        {
            
            cout<<"flag_array[0] = "<<flag_array[0]<<endl;
            twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
            send_vel1_Arc_path.publish(twist);
            flag_array[0] = 1;
        }
        cout<<"(abs(Cur_Distance-abs(Distance)) = "<<abs(Cur_Distance-abs(Distance))<<endl;
        loop_rate_Arc_path.sleep();
    }
}
/*
 函数名：void Arc_path_left 
 功能  :实现半径为R的圆 
 传入参数4个：
            R:圆弧的半径
            L：两个轮子间的距离 单位cm
            speed:为圆内侧速度 前进为正 后退为负 
            Distance:弧线端点的直线距离
返回值：无
实例子：Arc_path_left(0.707*0.75,41.0,0.1,0.45*3.141*0.5,85);
*/
void Arc_path_left(float R,float L,float speed,float Distance,float Set_Angle)
{
    cout<<"圆弧内侧速度不变的弧线运动"<<endl;
    ros::Rate loop_rate_Arc_path(10);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    Eigen::Quaterniond q_Arc_path_test;
    float Rotate_1=0;
    float R_1=0;
    float Cur_Distance = 0;
    float K = 0;
    float Angle_Arc = 0;
    L = L/100.0;
    K = L*speed/(2*R-L);//左右轮子速度和车体速度的差
    geometry_msgs::Twist twist;
	twist.linear.x =speed+K*L;twist.linear.y = 0; twist.linear.z = 0;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K;//K
    send_vel1_Arc_path.publish(twist);
    ros::spinOnce();
    cout<<"Distance = "<<Distance<<"flag_array[2] = "<<flag_array[2]<<endl;
    while((flag_array[2]==0)&&ros::ok())
    {
        Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
        q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
        q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
        q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
        q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
        cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Arc = euler_linear[2]*57.29578;
        cout<<"Angle_Arc = "<<Angle_Arc<<endl;
       // if(abs(Angle_Arc)<=85)
       if(abs(Angle_Arc)<=Set_Angle)
        {
            flag_array[2] = 0;
            ros::Time begin = ros::Time::now();
            ros::Duration(0.1).sleep();
            ros::Time end = ros::Time::now();
            ros::Duration end_be = end - begin;
            Cur_Distance = Cur_Distance+abs(speed)*(end_be.toSec());
            geometry_msgs::Twist twist;
            twist.linear.x =speed+K;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K
            send_vel1_Arc_path.publish(twist);
        }
        else
        {
            cout<<"flag_array[2] = "<<flag_array[2]<<endl;
            twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
            send_vel1_Arc_path.publish(twist);
            flag_array[2] = 1;
        }
        cout<<"(abs(Cur_Distance-abs(Distance)) = "<<abs(Cur_Distance-abs(Distance))<<endl;
        loop_rate_Arc_path.sleep();
    }
}

/*
 函数名：void Arc_path_right
 传入参数4个：
            R:圆弧的半径
            L：两个轮子间的距离 单位cm
            speed:为圆外侧速度 前进为正 后退为负 
            Distance:弧线端点的直线距离
返回值：无
例子  ：    Arc_path_right(0.707*0.75,41.0,0.2,0.45*3.141*0.5,85);
*/
void Arc_path_right(float R,float L,float speed,float Distance,float Set_Angle)
{
    cout<<"圆弧外侧速度不变的弧线运动"<<endl;
    ros::Rate loop_rate_Arc_path(100);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    Eigen::Quaterniond q_Arc_path_test;
    float Rotate_1=0;
    float R_1=0;
    float Cur_Distance = 0;
    float K = 0;
    float Angle_Arc = 0;
    float adjust_Arc = 0;
    float atan_A = 0;
    flag_array[3] = 0;
    L = L/100.0;
    K = speed/(2*R+L)*1.095;//左右轮子速度和车体速度的差
    cout<<"K1 = "<<K<<endl;
    geometry_msgs::Twist twist;
	//twist.linear.x =speed*1.095-K;twist.linear.y = 0; twist.linear.z = 0;
	//twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K 角速度
    //send_vel1_Arc_path.publish(twist);
    ros::spinOnce();
    cout<<"Distance = "<<Distance<<"flag_array[3] = "<<flag_array[3]<<endl;
    oFile_init<< "section Arc ahead KKKKK"<< " "<<endl;
    while((flag_array[3]==0)&&ros::ok())
    {
        ros::Time begin1 = ros::Time::now();
        run_get_pose();
        Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
        q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
        q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
        q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
        q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
        cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Arc = euler_linear[2]*57.29578;
        cout<<"Angle_Arc = "<<Angle_Arc<<endl;
        //speed = speed - abs(Set_Angle-Angle_Arc)*0.002;
        cout<<"speed = "<<speed<<endl;
        oFile_init<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<" "<<"A"<<" "<<Angle_Arc<<endl;

        //K = L*speed/(2*R+L);
       //amcl_get_pose();//打印AMCL坐标值
       if(abs(Angle_Arc)<=Set_Angle)
        {
        /*
           // if(abs(Angle_Arc)>=80)
           // if(sqrt(pow((1-flag_array[8]),2)+pow((1-flag_array[8]),2))>0.1)
          // if(abs(Angle_Arc)>=70)
            //{
 
                   // flag_array[11] = 90-atan((1-flag_array[9])/(1-flag_array[8]))*57.29578 -Angle_Arc;
                   // atan_A = atan((1-flag_array[9])/(1-flag_array[8]))*57.29578;
                   // cout<<"atan = "<<atan((1-flag_array[9])/(1-flag_array[8]))*57.29578<<"flag_array[11] = "<<flag_array[11]<<endl;
                   // oFile<< "atan "<< " "<<atan((1-flag_array[9])/(1-flag_array[8]))*57.29578<<" "<<"Angle_Arc "<<" "<<Angle_Arc<<"f11 "<<" "<<flag_array[11]<<endl;
                    
                  //  adjust_Arc  =  flag_array[11]*0.0001;
                  //  cout<<"adjust_Arc = "<<adjust_Arc<<endl;
                  //  //cout<<

           // }
           // oFile<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<" "<<"A"<<" "<<Angle_Arc<<" "<<"atan_A"<<" "<<atan_A;
        */
            flag_array[3] = 0;
            geometry_msgs::Twist twist;
            twist.linear.x =speed*1.095-K;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K
            send_vel1_Arc_path.publish(twist);
        }
        else
        {
            cout<<"flag_array[3] = "<<flag_array[3]<<endl;
 //           twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
 //           twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
 //           send_vel1_Arc_path.publish(twist);
            flag_array[3] = 1;
        }
        loop_rate_Arc_path.sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin1;
        cout<<"end_be.toSec() = "<<end_be.toSec()<<endl;
    }
}

/*
 函数名：void Arc_path_right
 传入参数4个：
            R:圆弧的半径
            L：两个轮子间的距离 单位cm
            speed:为圆外侧速度 前进为正 后退为负 
            Distance:弧线端点的直线距离
返回值：无
例子  ：    Arc_path_right_goahdead(0.707*0.75,41.0,0.2,0.45*3.141*0.5,85);
*/
void Arc_path_right_goahdead(float R,float L,float speed,float Distance,float Set_Angle)
{
    cout<<"圆弧外侧速度不变的弧线运动"<<endl;
    ros::Rate loop_rate_Arc_path(100);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    Eigen::Quaterniond q_Arc_path_test;
    float Rotate_1=0;
    float R_1=0;
    float Cur_Distance = 0;
    float K = 0;
    float Angle_Arc = 0;
    float adjust_Arc = 0;
    float atan_A = 0;
    flag_array[3] = 0;
    L = L/100.0;
    K = speed/(2*R+L)*1.095;//左右轮子速度和车体速度的差
    cout<<"K1 = "<<K<<endl;
    geometry_msgs::Twist twist;
	//twist.linear.x =speed*1.095-K;twist.linear.y = 0; twist.linear.z = 0;
	//twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K 角速度
    //send_vel1_Arc_path.publish(twist);
    ros::spinOnce();
    cout<<"Distance = "<<Distance<<"flag_array[3] = "<<flag_array[3]<<endl;
    oFile_init<< "section Arc ahead "<< " "<<endl;
    while((flag_array[3]==0)&&ros::ok())
    {
        ros::Time begin1 = ros::Time::now();
        run_get_pose();
        Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
        q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
        q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
        q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
        q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
        cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Arc = euler_linear[2]*57.29578;
        cout<<"Angle_Arc = "<<Angle_Arc<<endl;
        //speed = speed - abs(Set_Angle-Angle_Arc)*0.002;
        cout<<"speed = "<<speed<<endl;
        oFile_init<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<" "<<"A"<<" "<<Angle_Arc<<endl;

        //K = L*speed/(2*R+L);
       //amcl_get_pose();//打印AMCL坐标值
       if(abs(Angle_Arc)>=Set_Angle)
        {
        /*
           // if(abs(Angle_Arc)>=80)
           // if(sqrt(pow((1-flag_array[8]),2)+pow((1-flag_array[8]),2))>0.1)
          // if(abs(Angle_Arc)>=70)
            //{
 
                   // flag_array[11] = 90-atan((1-flag_array[9])/(1-flag_array[8]))*57.29578 -Angle_Arc;
                   // atan_A = atan((1-flag_array[9])/(1-flag_array[8]))*57.29578;
                   // cout<<"atan = "<<atan((1-flag_array[9])/(1-flag_array[8]))*57.29578<<"flag_array[11] = "<<flag_array[11]<<endl;
                   // oFile<< "atan "<< " "<<atan((1-flag_array[9])/(1-flag_array[8]))*57.29578<<" "<<"Angle_Arc "<<" "<<Angle_Arc<<"f11 "<<" "<<flag_array[11]<<endl;
                    
                  //  adjust_Arc  =  flag_array[11]*0.0001;
                  //  cout<<"adjust_Arc = "<<adjust_Arc<<endl;
                  //  //cout<<

           // }
           // oFile<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<" "<<"A"<<" "<<Angle_Arc<<" "<<"atan_A"<<" "<<atan_A;
        */
            flag_array[3] = 0;
            geometry_msgs::Twist twist;
            twist.linear.x =speed*1.095-K;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K
            send_vel1_Arc_path.publish(twist);
        }
        else
        {
            cout<<"flag_array[3] = "<<flag_array[3]<<endl;
 //           twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
 //           twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
 //           send_vel1_Arc_path.publish(twist);
            flag_array[3] = 1;
        }
        loop_rate_Arc_path.sleep();
        ros::Time end = ros::Time::now();
        ros::Duration end_be = end - begin1;
        cout<<"end_be.toSec() = "<<end_be.toSec()<<endl;
    }
}

/*
 函数名：void Arc_path_right_back
 传入参数4个：
            R:圆弧的半径
            L：两个轮子间的距离 单位cm
            speed:为圆外侧速度 前进为正 后退为负 
            Distance:弧线端点的直线距离
返回值：无
例子  ：    Arc_path_right(0.707*0.75,41.0,0.2,0.45*3.141*0.5,85);
*/
void Arc_path_right_back(float R,float L,float speed,float Distance,float Set_Angle)
{
    cout<<"圆弧外侧速度不变的弧线运动"<<endl;
    ros::Rate loop_rate_Arc_path(10);
    ros::NodeHandle Arc_path;
    ros::Publisher send_vel1_Arc_path = Arc_path.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Arc_path_test_linear;
    tf::StampedTransform transform_Arc_path_test;
    Eigen::Quaterniond q_Arc_path_test;
    float Rotate_1=0;
    float R_1=0;
    float Cur_Distance = 0;
    float K = 0;
    float Angle_Arc = 0;
    float adjust_Arc = 0;
    float atan_A = 0;
    flag_array[3] = 0;
    L = L/100.0;
    K = L*speed/(2*R+L)*1.095;//左右轮子速度和车体速度的差
    cout<<"K = "<<K<<endl;
    geometry_msgs::Twist twist;
	//twist.linear.x =speed*1.095-K;twist.linear.y = 0; twist.linear.z = 0;
	//twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K 角速度
    //send_vel1_Arc_path.publish(twist);
    ros::spinOnce();
    cout<<"Distance = "<<Distance<<"flag_array[3] = "<<flag_array[3]<<endl;
    oFile_init<< "section Arc back "<< " "<<endl;
    while((flag_array[3]==0)&&ros::ok())
    {
        run_get_pose();
        Arc_path_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Arc_path_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Arc_path_test);
        q_Arc_path_test.x() = transform_Arc_path_test.getRotation().getX();
        q_Arc_path_test.y() = transform_Arc_path_test.getRotation().getY();
        q_Arc_path_test.z() = transform_Arc_path_test.getRotation().getZ();
        q_Arc_path_test.w() = transform_Arc_path_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Arc_path_test.toRotationMatrix().eulerAngles(0, 1, 2);
        cout << "x = "<<transform_Arc_path_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Arc = euler_linear[2]*57.29578;
        cout<<"Angle_Arc = "<<Angle_Arc<<endl;
        //speed = speed - abs(Set_Angle-Angle_Arc)*0.002;
        cout<<"speed = "<<speed<<endl;
        oFile_init<< "X"<< " "<<flag_array[8]<<" "<<"Y"<<" "<<flag_array[9]<<" "<<"A"<<" "<<Angle_Arc<<endl;

        //K = L*speed/(2*R+L);
       //amcl_get_pose();//打印AMCL坐标值
       if(Angle_Arc>=Set_Angle)
        {
            flag_array[3] = 0;
            ros::Time begin = ros::Time::now();
            ros::Duration(0.1).sleep();
            ros::Time end = ros::Time::now();
            ros::Duration end_be = end - begin;
            Cur_Distance = Cur_Distance+abs(speed)*(end_be.toSec());
            geometry_msgs::Twist twist;
            twist.linear.x =speed*1.095-K;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =K/L;//K
            send_vel1_Arc_path.publish(twist);
        }
        else
        {
            cout<<"flag_array[3] = "<<flag_array[3]<<endl;
           // twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
           // twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
           // send_vel1_Arc_path.publish(twist);
            flag_array[3] = 1;
        }
        cout<<"(abs(Cur_Distance-abs(Distance)) = "<<abs(Cur_Distance-abs(Distance))<<endl;
        loop_rate_Arc_path.sleep();
    }
    oFile_init<< "section Arc back "<< " "<<endl;
}


void Robot_Rotation_180(float L,float speed)
{
    cout<<"自转运动"<<endl;
    ros::Rate loop_rate_Robot_Rotation(10);
    ros::NodeHandle Rotation;
    ros::Publisher send_vel1_Rotation = Rotation.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Rotation_test_linear;
    tf::StampedTransform transform_Rotation_test;
    Eigen::Quaterniond q_Rotation_test;
    L = L/100.0;
    float Angle_Rotation = 0;
    float Pre_Angle_Rotation = 0;
    char direction = 0;
    flag_array[1] = 0;
    while((flag_array[1]==0)&&ros::ok())
    {
        Rotation_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Rotation_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Rotation_test);
        q_Rotation_test.x() = transform_Rotation_test.getRotation().getX();
        q_Rotation_test.y() = transform_Rotation_test.getRotation().getY();
        q_Rotation_test.z() = transform_Rotation_test.getRotation().getZ();
        q_Rotation_test.w() = transform_Rotation_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Rotation_test.toRotationMatrix().eulerAngles(0, 1, 2);
        cout << "x = "<<transform_Rotation_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Rotation = euler_linear[2]*57.29578;//0-360度
        cout<<"Angle_Rotation = "<<Angle_Rotation<<"  Pre_Angle_Rotation = "<<Pre_Angle_Rotation<<endl;

        if(Angle_Rotation>=-90)
        {
            geometry_msgs::Twist twist;
            twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =speed/L;
            send_vel1_Rotation.publish(twist);
        }
        else
        {
            cout<<"flag_array[1] = "<<flag_array[1]<<endl;
             geometry_msgs::Twist twist;
            twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
            send_vel1_Rotation.publish(twist);
            flag_array[1] = 1;
        }
        loop_rate_Robot_Rotation.sleep();
    }
}

void Robot_Rotation_180_X(float L,float speed)
{
    cout<<"自转运动"<<endl;
    ros::Rate loop_rate_Robot_Rotation(10);
    ros::NodeHandle Rotation;
    ros::Publisher send_vel1_Rotation = Rotation.advertise<geometry_msgs::Twist>("cmd_vel",1);//改为1
    tf::TransformListener Rotation_test_linear;
    tf::StampedTransform transform_Rotation_test;
    Eigen::Quaterniond q_Rotation_test;
    L = L/100.0;
    float Angle_Rotation = 0;
    float Pre_Angle_Rotation = 0;
    char direction = 0;
    flag_array[19] = 0;
    while((flag_array[19]==0)&&ros::ok())
    {
        Rotation_test_linear.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        Rotation_test_linear.lookupTransform("/odom", "/base_link", ros::Time(0), transform_Rotation_test);
        q_Rotation_test.x() = transform_Rotation_test.getRotation().getX();
        q_Rotation_test.y() = transform_Rotation_test.getRotation().getY();
        q_Rotation_test.z() = transform_Rotation_test.getRotation().getZ();
        q_Rotation_test.w() = transform_Rotation_test.getRotation().getW();
        Eigen::Vector3d euler_linear = q_Rotation_test.toRotationMatrix().eulerAngles(0, 1, 2);
        cout << "x = "<<transform_Rotation_test.getOrigin().x() << endl ;
        cout << "A = "<< euler_linear[2]*57.29578 << endl ;//得到的弧度值转化为角度值
        Angle_Rotation = euler_linear[2]*57.29578;//0-360度
        cout<<"Angle_Rotation = "<<Angle_Rotation<<"  Pre_Angle_Rotation = "<<Pre_Angle_Rotation<<endl;

        if((Angle_Rotation>=-178)&&(Angle_Rotation<=90))
        {
            geometry_msgs::Twist twist;
            twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =speed/L;
            send_vel1_Rotation.publish(twist);
        }
        else
        {
            cout<<"flag_array[19] = "<<flag_array[19]<<endl;
             geometry_msgs::Twist twist;
            twist.linear.x =0;twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =0;
            send_vel1_Rotation.publish(twist);
            flag_array[19] = 1;
        }
        loop_rate_Robot_Rotation.sleep();
    }
}
void Build_map()
{
     //pid.Kp=0.020;
     //pid.Ki=0.025;
     //pid.Kd=0.004;
     linear_motion(0.08,0.1,1,1);
     linear_motion(0.1,0.1,10,1);
     linear_motion(0.1,0.08,1,1);
     cout<<"完成直线行驶"<<endl;
    // Arc_path(1.0,41.0,0.08,3.141*0.8);//0.8
     cout<<"完成前进弧线行驶"<<endl;
     Send_stop();
     Send_stop();
     ros::Duration(5).sleep();
     flag_array[0] = 0;
    // Arc_path(1.0,41.0,-0.08,3.141*0.8);//0.8
     cout<<"完成后退弧线行驶"<<endl;
    // Send_stop();
    // Send_stop();
     linear_motion(-0.08,-0.1,-1,0);
     linear_motion(-0.1,-0.1,-10,0);
     linear_motion(-0.1,-0.0,-1,0);
     cout<<"结束任务"<<endl;
     Send_stop();
}
void robot_line_Arc_path()
{
     //linear_motion(0.1,0.25,1,1);
     //linear_motion(0.25,0.25,10.5,1);//11.2
     //linear_motion(0.25,0.25,1,1);
    // cout<<"完成直线行驶"<<endl;
     //Arc_path(1.0,41.0,0.1,3.141*0.75);//0.8 //内侧速度变小
     //Arc_path_test(0.707*0.75,41.0,0.25,0.45*3.141*0.5,85);//圆内侧速度不变
     //Arc_path_test(0.353,41.0,0.2,0.45*3.141*0.5,85);
     //Arc_path_right(0.707*0.75,41.0,0.25,0.45*3.141*0.5,85);
      Arc_path_right(1*0.9,41.0,0.2,0.45*3.141*0.5,85);
      Send_stop();
      ros::Duration(5).sleep();
     // flag_array[3] = 0;
     Arc_path_right_back(1*0.9,41.0,-0.2,0.45*3.141*0.5,0);
     cout<<"圆弧外侧速度不变的弧线运动"<<endl;
    
    // Arc_path_right(0.707*0.75,41.0,0.2,0.45*3.141*0.5,85);
     cout<<"完成前进弧线行驶"<<endl;
     Send_stop();
     Send_stop();
     //ros::Duration(5).sleep();
     //flag_array[0] = 0;
     //Arc_path(1.0,41.0,-0.1,3.141*0.75);//0.8
     //cout<<"完成后退弧线行驶"<<endl;
     //Send_stop();
     //Send_stop();
    // linear_motion(-0.1,-0.25,-1,0);
    // linear_motion(-0.25,-0.25,-10,0);
    // linear_motion(-0.25,-0.0,-1,0);
     cout<<"结束任务"<<endl;
     Send_stop();
}


int main1()
{
  //cout<<"Hello SLAM"<<endl;
 printHello();
 return 0;
}
