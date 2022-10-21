#ifndef SWERVECHASSIS_H
#define SWERVECHASSIS_H

#include "stm32h7xx.h"
#include "devices.h"
#include "pid.h"
#include "holder.h"
#include "load.h"
#define vx_Sence 0.04f  //0.02f
#define vy_Sence 0.06f  //0.03f
/**
  * @brief  ���ֵ��̽ṹ��
  */
typedef struct 
{
	struct
	{
		uint8_t Enable;    						//< ʹ��״̬			
		uint8_t isRefereeUpdating; 		//< ����ϵͳ�Ƿ��ڸ�������
		uint8_t DriveMode;						//< ����ģʽ
		uint8_t ChassisState;					//< ����״̬
	}Flag;
	
	struct 
	{
		float Target_Power_Sum[2];		
		BasePID_Object PowerPID[2];
		float Max_Power;		
    float speedfinalout[4];
    float anglefinalout[4];	
		float bili;
		float bilibili;
	}Power;
	
	struct 
	{
		Motor motor[4];						//< ���̵���ṹ��
		BasePID_Object RunPID[4];			//< �ٶȿ���PID����
		BasePID_Object FollowPID[1];			//< ���̸���PID����
	}Motors3508;
	
	struct 
	{
		Motor motor[4];						//< ���̵���ṹ��
		BasePID_Object TurnPID[4];			//< ת��Ƕȿ��ƽṹ��
	}Motors6020;
	
	//< ���Ƶ����˶�����Ҫ����������
	struct 
	{
		int16_t Vx;			//< ǰ���˶����ٶ�
		int16_t Vy;		  	//< �����˶����ٶ�
		int16_t Omega;		//< ��ת�Ľ��ٶ�
	  int16_t		DeltaVx;
	  int16_t		DeltaVy;
	  int16_t		DeltaOmega;
		struct 
		{
			float		Vx;
			float		Vy;
			float		Omega;
		}Sensitivity;
		
		int16_t ModuleOfSpeed;		//< �ٶ�������ģֵ
		float  AngleOfSpeed;		//< �ٶ������ĽǶ�
	}Movement;  
	
	struct 
	{
		int16_t Vx[4];
		int16_t Vy[4];
		float Angle[4];	
		float BestAngle[4];
		int16_t TargetEcd[4];
		float FeedbackAngle[4];
		int16_t SpeedNo[4];
		uint8_t SpeedChangeFlag[4];
	}Vectors;
	uint8_t SuperPowerMode;
}SwerveChassis;


/**
  * @brief  �޸Ķ���PID�����Ľṹ��
  */
typedef struct 
{	
	float SpeedKp, SpeedKi, SpeedKd;
	float TurnKp, TurnKi, TurnKd;
}PIDParameters;

extern int8_t rush_cut;
extern int8_t rush_flag;
extern uint8_t fly_flag;

/**
  * @brief  �޸Ķ��ֵ���PID�����Ľӿں���
  * @param[in]  chassis     ���̽ṹ��
  * @param[in]  pid     		����6020��3508�ĽǶȺ��ٶȿ���PID����
  */
void ChangeChassisPID(SwerveChassis* chassis, PIDParameters pid);


/**
  * @brief  ���ֵ��̳�ʼ��,�����˸�����������������ֵ��PID����
  * @param[in]  chassis     ���̽ṹ��
  * @param[in]  run_pid     3508����PID
  * @param[in]  turn_pid    6020ת��PID
  * @param[in]  canx	   		���ֹ��ص�CAN���߱�� ����CAN1��CAN2
  */
void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, CanNumber canx);


/**
  * @brief  		��ȡ���ջ��������ݣ����ݶ����˶��߼�����
	* @param[in]	chassis  ���̽ṹ��
	* @param[in]	rc_ctrl  ң�����ṹ��,���ڸ��¿�������
	* @param[in]	canAngle YAW�����������Ƕȣ�0��+-180�㣩
  */
void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Chassis_Attitude_Info* Swerve, float canAngle);

void SwerveAngleChassisInit(SwerveChassis* chassis,CanNumber canx);
/**
  * @brief  �ڶ������˶�ѧ����󱻵��ã�����3508��6020���������ֵ��������CAN���ݵ�0x200��0x1ff��������֡
	* @param[in]	chassis  ���̽ṹ��
  */
void SwerveChassisMotionControl(SwerveChassis* chassis , RC_Ctrl* rc_ctrl ,Holder_t* holder);


/**
	* @brief ���ݽ��ջ��Ƿ����ߣ��ж��Ƿ����������̿����źš�����0x1ff��0x200����CAN����֡�ķ������
	* @param[in]	chassis  ���̽ṹ��
	* @param[in]	rcCtrl   ң�����ṹ��,���ڶ�ȡң�������߱�־λ
	*/

void SwerveChassisSetFollowPID(SwerveChassis* chassis, BasePID_Object follow_pid);

void SwervePowerConrolInit(SwerveChassis* chassis, BasePID_Object base_pid, BasePID_Object power_pid);

#endif
