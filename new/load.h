#ifndef LOAD_H
#define LOAD_H
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "devices.h"
#include "vision.h"
//#define  OPEN_Compare 2100  //���ոǿ�
//#define  CLOSE_Compare 1200  //���ոǹ�
typedef struct  
{
	int8_t  Shoot_Mode;           //����ģʽ
	int8_t  Shoot_Target_Num;     //��ĵ���
	int16_t Target_Firing_Rate;   //����Ŀ���ٶ�
	int16_t Forward_Reverse_Flag; //��ͬ���ת�ķ���ͬ�������߼�ֻ֧����ת�������Ҫһ����־λ���ñ������ɸ�ֵ
	uint32_t First_Single_Flag;   //��������ģʽʱ��ֵΪ0�����ڽ���������λ�ж�
	uint8_t  Finish_Flag;         //δ��
	uint16_t Shoot_Gap;           //����ʱ������������ֵʱת��ģʽΪ����
	uint16_t Shoot_Gap_Limit;     //��������
	int32_t  Shoot_sum_cut;
	uint8_t SuperHeatMode;
	volatile long int Shoot_Target_Sum;      //Ŀ�귢�䵯������
	volatile long int Shoot_Already_Sum;     //���䵯��������δ��
	volatile long int Shoot_Bullet_Sum;      //����ϵͳ��ⷢ������
	volatile long int Last_Shoot_Bullet_Sum; //��һʱ�̲���ϵͳ��ⷢ������
  uint8_t Friction_Status_Flag;            //����Ħ���ֱ�־λ��1Ϊ��
	uint8_t BulletCap_Status_Flag;           //�������ոǱ�־λ��1Ϊ��
	uint8_t Crowd_Flag;
	uint8_t Crowd_back_Flag;                 //��ת��־λ
	uint8_t Stop_Back_Flag;                 
		struct 
	{
		Motor motor[4];						//< ���̵���ṹ��
		BasePID_Object RunPID[4];			//< �ٶȿ���PID����
	}Motors3508;
			struct 
	{
		Motor motor[1];						//< ���̵���ṹ��
		BasePID_Object RunPID[4];			//< �ٶȿ���PID����
	}Motors2006;
	float loadout[3];
}Shoot_Info;
typedef struct            //Ħ���ֲ�������Ϣ
{
	float Target_Speed[3];       //�ƶ���Ŀ���ٶ�       
	int16_t Feedback_Speed[3];   //���ӷ����ٶ�
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
	float Kp_Pid;//����ϵ��
	float Ki_Pid;//����ϵ��
	float Kd_Pid;//΢��ϵ��  
	float E0_Pid;//��ֵ
	float Last_E0_Pid;
	float Last_Last_E0_Pid;
	float Ep_Part_Pid;//�������ֿ�����
	float Ei_Part_Pid;//���ֲ��ֿ�����
	float Ed_Part_Pid;//΢�ֲ��ֿ�����
	float Out_Pid;//������ֵ
	float Target_Pid; //Ŀ��ֵ
	float Feedback_Pid;//����ֵ
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