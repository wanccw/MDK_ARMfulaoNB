#ifndef INIT_H
#define INIT_H
#include "stm32h7xx_hal.h"
#include "fdcan.h"
#define abs(x) ((x)>0? (x):(-(x)))
#define Yaw_Integ_Sense 0.009f
#define Pitch_Integ_Sense 0.0001f
#define BMI088_Wait_Time 3000

#define FRICTION_SPEED_LOW 4850
#define FRICTION_SPEED_MIDLE 6100
#define FRICTION_SPEED_HIGH 7350
void System_Init();
extern int32_t Clock_time;
#define ROCK_TIME_SIN 45.0f	 //�޸Ĵ�ֵ����ҡ�ڵ��ٶ�
#define  Yaw_Base_Can_Angle    650 //��̨����ʱyaw�ı�����ֵ
#define  Pitch_Base_Can_Angle 2893    //��̨����ʱpitch�ı�����ֵ
#define YAW_ENCODER_FULL_RANGE  8192 //������������
#define PITCH_ENCODER_FULL_RANGE 8192
#define Chassis_ENCODER_FULL_RANGE 8192
#define Chassis_Base_Can_Angle1 6821
#define Chassis_Base_Can_Angle2 7583
#define Chassis_Base_Can_Angle3 5480
#define Chassis_Base_Can_Angle4 4921
#define K_Code_Pitch  0.043945f   //  360/8192
#define K_Code_Yaw  0.043945f   //  360/8192
#define FRI_LEFT 0					// in "Motor_Booster.Target_Speed[0]" staies the target angle speed for left friction
#define FRI_RIGHT 1					//
#define LOADER 2						// 
#define FIRE_ONE_ONETIME 1
#define FIRE_THREE_ONETIME 3
#define  PITCH_NEGTIVE_LIMIT_ANGLE 39.0f		//��̨�·��Ƕ�����
#define  PITCH_POSITIVE_LIMIT_ANGLE -30.0f	//��̨�Ϸ��Ƕ�����

#endif