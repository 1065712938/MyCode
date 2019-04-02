#ifndef LIBHELLOSLAM_H_
#define LIBHELLOSLAM_H_
void printHello();
void Send_stop();
void Send_goahead(float vel);
void Verification_vel(float vel,int time_s);
void PID_init();
void run_get_pose();
void amcl_get_pose();
extern float g_odom_x;
extern float g_odom_y;
extern int Arc_flag;
extern float ranges[360];
extern float flag_array[20];
extern std::ofstream oFile_init;
float PID_realize(float Target_value,float Real_time_value);
void amcl_linear_back1(float V0,float V1,double Distance,char flag);
void Track_pose(float L,float K,float x,float y,float x1,float y1);
void Robot_Rotation(float L,float speed,float Set_Angle);
void amcl_linear_Y_back(float V0,float V1,double Distance,char flag);
void amcl_linear_back(float V0,float V1,double Distance,char flag);
void amcl_linear_back_Random(float V0,float V1,double Distance,char flag);
void amcl_linear_back_Avoidance(float V0,float V1,double Distance,char flag,int state,int mode,float speed_obs);
void amcl_linear_back_Random_Avoidance02(float V0,float V1,double Distance,char flag);
void linear_motion(float V0,float V1,float Distance,char flag);
void linear_motion_goahead(float V0,float V1,float Distance,char flag,float Pid_set);
void Arc_path(float R,float L,float speed,float Distance);
void Arc_path_left(float R,float L,float speed,float Distance,float Set_Angle);
void Arc_path_right(float R,float L,float speed,float Distance,float Set_Angle);
void Arc_path_right_goahdead(float R,float L,float speed,float Distance,float Set_Angle);
void Arc_path_right_back(float R,float L,float speed,float Distance,float Set_Angle);
float get_run_speed(double C_Pose,double G_Pose,float Cur_speed,float max_speed,float speed_inc);

void Robot_Rotation_180(float L,float speed);
void Robot_Rotation_180_X(float L,float speed);
void Buffer_emergency_stop(float vel,int num);
void robot_line_Arc_path();
void Build_map();
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
#endif
