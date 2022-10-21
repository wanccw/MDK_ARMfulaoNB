#ifndef HOLD_H
#define HOLD_H
#include "stm32h7xx_hal.h"
#include "devices.h"
#include "pid.h"
struct Holder_Motor_Info
{	
	float Target_Angle;
	float Bmi088_Angle;
	float MPU6050_Angle;
	float Last_MPU6050_Angle;
	float MPU6050_Angle_Change;
	int16_t Can_Angle_Raw;
	int16_t Can_Angle_Add;
	float Can_Angle;
	float Last_Can_Angle;
	float Can_Angle_Change;
	float Bmi088_Angle_speed;
	float MPU6050_Angle_speed;
	float MPU6050_Angle_speed1;
	float Can_Angle_speed;	
	int16_t Can_Speed_Feedback;
	float Sensitivity;
	float MouseSensitivity;
	float Angle_error;
};
typedef struct 
{
  struct Holder_Motor_Info Pitch;
	struct Holder_Motor_Info Yaw;
	struct 
	{
		Motor motor[2];						//< 底盘电机结构体
		BasePID_Object turnPID[6];		//< 转向角度控制结构体
		float FeedbackAngle[2];
		float HolderCanAngle[2];
	}Motors6020;
  float cruise;//巡航
	uint8_t Direction_Flag;
	uint8_t Direction_cut;
	uint8_t Reset_OK_Flag;
	uint8_t ESC_Reset_flag;
}Holder_t;

extern float up_limit;
extern float down_limit;

void HolderYawChassisInit(Holder_t* holder, CanNumber canx);
void HolderReset(Holder_t* holder);
void HolderInit(Holder_t* holder, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object pitch_angle_pid , BasePID_Object pitch_speed_pid,BasePID_Object yaw_reset_pid,BasePID_Object pitch_reset_pid,CanNumber canx);
void HolderGetRemoteData(Holder_t* holder, RC_Ctrl* rc_ctrl);
//void HolderChassisInit(Holder_t* holder, BasePID_Object yawpid , CanNumber canx);
//void Holder1ChassisInit(Holder_t* holder, BasePID_Object pitchpid , CanNumber canx);
#endif
