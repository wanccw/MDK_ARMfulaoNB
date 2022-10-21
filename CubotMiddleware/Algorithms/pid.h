#ifndef __PID_H_
#define __PID_H_

#include "stm32h7xx_hal.h"



/**
  * @brief  ����PID 
  */
typedef struct 
{	
	float Kp, Ki, Kd;
	float Error;
	float KpPart, KiPart, KdPart;
	float Out;
	float KiPartDetachment;
	float LastError;
}BasePID_Object;


/**
  * @brief  ˫��PID
  */
typedef struct 
{
	BasePID_Object* ShellPID;
	BasePID_Object* CorePID;
}DualPID_Object;
 

/**
  * @brief  		�޷�
  * @param[in]	input		������
  * @param[in]	output	�����
  */
float AmplitudeLimit(float input,float amplitude);


/**
  * @brief ����PID��ʼ��
  */
void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach);


/**
  * @brief ˫��PID��ʼ��
  */
void DualPID_Init(DualPID_Object* dual_pid, float shell_kp, float shell_ki, float shell_kd, float core_kp, float core_ki, float core_kd, float detach);



/**
  * @brief �������������ٶȿ���
  */
float BasePID_SpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);



/**
  * @brief �������������ٶȿ���, ���ٶȲ�����IMU���ݣ��������������нǶȿ���
  */
int32_t BasePID_AngleControlWithoutIMU(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);


/**
  * @brief ��������΢�ֽǶȿ���, ���ٶȲ�����IMU���ݣ��������������нǶȿ���
  */
int32_t BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);
int32_t BasePID_AngleControlFollow(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);
float BasePID_BaseControl(BasePID_Object* base_pid, float Chassis_Power_Buffer, float Chassis_Power);
int32_t BasePID_HoldControl(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);
float BasePID_PowerControl(BasePID_Object* base_pid, float Chassis_Power_Buffer, float Chassis_Power);
int32_t BasePID_PitchAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle);
int32_t BasePID_PitchSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);
int32_t BasePID_YawAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle);
int32_t BasePID_YawSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);
#endif

