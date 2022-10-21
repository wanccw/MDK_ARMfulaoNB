#ifndef MECANUMCHASSIS_H
#define MECANUMCHASSIS_H

#include "stm32h7xx.h"
#include "devices.h"
#include "pid.h"


typedef enum 
{
	Follow = 0x00U,    	 //< ����
	Rock   = 0x01U,		 //< ҡ��
	Spin   = 0x02U		 //< ��ת
}ChassisState;


/**
  * @brief  �����ķ�ֵ��̽ṹ��
  */
typedef struct 
{
	struct
	{
		uint8_t Enable;    					//< ʹ��״̬			
		uint8_t isRefereeUpdating; 			//< ����ϵͳ�Ƿ��ڸ�������
		uint8_t DriveMode;					//< ����ģʽ
		uint8_t ChassisState;				//< ����״̬
	}Flag;
	
	struct 
	{
		uint32_t Target_Power_Sum;
  	BasePID_Object PowerPID;
		uint8_t Max_Power;				
	}Power;
	
	struct 
	{
		Motor motor[4];
		BasePID_Object RunPID[4];
  	BasePID_Object FollowPID;			
	}Motors;
	
	//< ���Ƶ����˶�����Ҫ����������
	struct 
	{
		int16_t Vx;			//< ǰ���˶����ٶ�
		int16_t Vy;		    //< �����˶����ٶ�
		int16_t Vomega;		//< ��ת�Ľ��ٶ�
		float		Vx_Sensitivity;
		float		Vy_Sensitivity;
		float		Vomega_Sensitivity;		
	}Movement;
	
}MecanumChassis;



/**
  * @brief  		���ֵ��̳�ʼ�������������ĸ����̵�������ҿ�����ͬ��PID����
  * @param[in]  chassis     ���̽ṹ��
  * @param[in]  canx	   		���̹��ص�CAN���߱�� ����CAN1��CAN2
  * @param[in]  run_pid     3508����PID
  * @param[in]  base_id     ���Ͻǵ���Ľ���id,����0x201
  */
void MecanumChassisInit(MecanumChassis* chassis, BasePID_Object run_pid, CanNumber canx);


/**
  * @brief  ���ֵ��̴�ң�������¿������ݣ����ҽ���ģʽѡ��
  * @param[in]  chassis     ���̽ṹ��
  * @param[in]  rc_ctrl	   	���ջ��ṹ�壬��ÿ����ź�
  */
void MecanumChassisGetRemoteData(MecanumChassis* chassis, RC_Ctrl* rc_ctrl);


/**
  * @brief  ���ֵ��̴Ӳ���ϵͳ�����¿�������
  */
void MecanumChassisGetRefereeData();


/**
  * @brief  ���ֵ������˶�ѧ���㣬Inverse Kinematics
  * @param[in]  chassis     ���̽ṹ��
	* @param[in]	canAngle       YAW�����������Ƕȣ�0��+-180�㣩
  */
void MecanumChassisMotionControl(MecanumChassis* chassis, float canAngle);


/**
  * @brief  ���ֵ��̷��͵����������
	* @param[in]	chassis  ���̽ṹ��
	* @param[in]	rcCtrl   ң�����ṹ��,���ڶ�ȡң�������߱�־λ
  */
void MecanumChassisOutputControl(MecanumChassis* chassis, RC_Ctrl rcCtrl);



#endif


