#ifndef LOAD_H
#define LOAD_H
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "devices.h"
#include "vision.h"
//#define  OPEN_Compare 2100  //弹舱盖开
//#define  CLOSE_Compare 1200  //弹舱盖关
typedef struct  
{
	int8_t  Shoot_Mode;           //发射模式
	int8_t  Shoot_Target_Num;     //打的弹数
	int16_t Target_Firing_Rate;   //决定目标速度
	int16_t Forward_Reverse_Flag; //不同电机转的方向不同，拨弹逻辑只支持正转，因此需要一个标志位将该变的量变成负值
	uint32_t First_Single_Flag;   //进行连发模式时赋值为0，用于结束连发后复位判断
	uint8_t  Finish_Flag;         //未用
	uint16_t Shoot_Gap;           //连射时递增，超过限值时转换模式为单发
	uint16_t Shoot_Gap_Limit;     //连射限制
	int32_t  Shoot_sum_cut;
	uint8_t SuperHeatMode;
	volatile long int Shoot_Target_Sum;      //目标发射弹丸总数
	volatile long int Shoot_Already_Sum;     //已射弹丸总数，未用
	volatile long int Shoot_Bullet_Sum;      //裁判系统检测发弹总数
	volatile long int Last_Shoot_Bullet_Sum; //上一时刻裁判系统检测发弹总数
  uint8_t Friction_Status_Flag;            //开启摩擦轮标志位，1为开
	uint8_t BulletCap_Status_Flag;           //开启弹舱盖标志位，1为开
	uint8_t Crowd_Flag;
	uint8_t Crowd_back_Flag;                 //反转标志位
	uint8_t Stop_Back_Flag;                 
		struct 
	{
		Motor motor[4];						//< 底盘电机结构体
		BasePID_Object RunPID[4];			//< 速度控制PID参数
	}Motors3508;
			struct 
	{
		Motor motor[1];						//< 底盘电机结构体
		BasePID_Object RunPID[4];			//< 速度控制PID参数
	}Motors2006;
	float loadout[3];
}Shoot_Info;
typedef struct            //摩擦轮拨弹盘信息
{
	float Target_Speed[3];       //移动的目标速度       
	int16_t Feedback_Speed[3];   //轮子反馈速度
	float Angle_Plate;
	float Angle_Target;
	float Angle_Target_Last;	
	float Target_Angle;
	float ThreeAngle;
	float Angle;
	int16_t a;
	float LatsAngle_Target;
	uint16_t  Inversion_Cnt;
	float Current_1[4];
	float delta_angle;
	float wrb_left_speed;
	float wrb_right_speed;
	float wrb_feed_back_speed[3];
}Friction_Load_Info;


typedef struct
{
//	float Target_Angle;
//	float Feedback_Angle;
//	float Target_Speed;
//	float Feedback_Speed;
	uint8_t Roll_Flag;
	uint8_t Roll_Flag1;
	uint8_t Rock_Flag;
	uint8_t Remake_Flag;
	uint8_t Chassis_State;
	float Chassis_Power;
	float Chassis_Power_Buff;
	float Chassis_Out_Sum;
	float Chassis_Allowed_Out;
}Chassis_Attitude_Info;
 struct Pid_Info
{	
	float Kp_Pid;//比例系数
	float Ki_Pid;//积分系数
	float Kd_Pid;//微分系数  
	float E0_Pid;//差值
	float Last_E0_Pid;
	float Last_Last_E0_Pid;
	float Ep_Part_Pid;//比例部分控制量
	float Ei_Part_Pid;//积分部分控制量
	float Ed_Part_Pid;//微分部分控制量
	float Out_Pid;//最后输出值
	float Target_Pid; //目标值
	float Feedback_Pid;//反馈值
};
extern float delta_angle;;
extern Shoot_Info Shoot;
extern Friction_Load_Info Booster;
extern Friction_Load_Info Motor_Booster;
extern uint16_t Friction_Start_Cnt;
extern struct Pid_Info Pid_Load;
extern struct Pid_Info Pid_Friction;
extern float Number_load[4];
void Steering_Engine_Control(RC_Ctrl* rc_ctrl,Shoot_Info* shoot);
void Friction_Load_Fire_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,Trace* Info_Vision);
void Steering_Engine_Setcompare(uint16_t Compare);
void LoadInit(Shoot_Info* shoot, BasePID_Object friction_pid, BasePID_Object load_pid, CanNumber canx);
void Update_Loader_Target_Angle(Shoot_Info* shoot ,Friction_Load_Info* booster);
void Update_Loader_Target_Speed(Shoot_Info* shoot ,Friction_Load_Info* booster,float Target_Angle, float Feedback_Angle);
void Loader_Jam_Deal(Shoot_Info* shoot ,Friction_Load_Info* booster);
void Friction_Shoot_Heat_Control_Deal(RC_Ctrl* rc_ctrl);
void Shoot_Fire_Mode_Control();
void Booster_Task(Shoot_Info* shoot,Friction_Load_Info* booster);
float Friction_Load_Pid_Control(BasePID_Object* base_pid,float Feedback,int16_t target,uint8_t who);
#endif