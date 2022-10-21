#include "mecanum_chassis.h"

#define AtR 0.0174532f	              //<  3.1415 /180 �Ƕ��� ת��Ϊ������	


/**
  * @brief  ���ֵ��̳�ʼ�������������ĸ����̵�������ҿ�����ͬ��PID������
  */
void MecanumChassisInit(MecanumChassis* chassis, BasePID_Object run_pid, CanNumber canx)
{
	for(int i = 1;i <= 4; i++)  //< ��ʼ���ĸ�����Ͷ�Ӧ��pid�ṹ��
	{
		MotorInit(&chassis->Motors.motor[i - 1], 0, Motor3508, canx, (0x200 + i)); 	//< ���ֵ��̵Ĵ�����Ĭ�Ϲ�����0x200�ϵ�
		BasePID_Init(&chassis->Motors.RunPID[i - 1], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	}	
	chassis->Movement.Vomega_Sensitivity = 1;
	chassis->Movement.Vx_Sensitivity     = 1;
	chassis->Movement.Vy_Sensitivity     = 1;
}


/**
  * @brief  ���õ��̸����PID����
  */
void MecanumChassisSetFollowPID(MecanumChassis* chassis, BasePID_Object follow_pid)
{
		BasePID_Init(&chassis->Motors.FollowPID, follow_pid.Kp, follow_pid.Ki, follow_pid.Kd, follow_pid.KiPartDetachment);
}



/**
  * @brief  ���ֵ��̴�ң�������¿������ݣ����ҽ���ģʽѡ��
  */
void MecanumChassisGetRemoteData(MecanumChassis* chassis, RC_Ctrl* rc_ctrl)
{
	chassis->Movement.Vx      = (rc_ctrl->rc.ch1 - 1024) * chassis->Movement.Vx_Sensitivity;	
	chassis->Movement.Vy		  = (rc_ctrl->rc.ch0 - 1024) * chassis->Movement.Vy_Sensitivity;	
	chassis->Movement.Vomega  = (rc_ctrl->rc.ch2 - 1024) * chassis->Movement.Vomega_Sensitivity; 
}



/**
  * @brief  ���ֵ������˶�ѧ���㣬Inverse Kinematics ,����chassis�ṹ���е�movement�ṹ�����ת�١�
  * @notec  �������δ���й��ʿ��Ƶĵ���������д��Motor�ṹ���С� 
  */
void MecanumChassisSetSpeed(MecanumChassis* chassis, float canAngle)
{
	float angle = canAngle * AtR;
	float rotated_vy = (chassis->Movement.Vx * sin(angle) + chassis->Movement.Vy * cos(angle));
	float rotated_vx = (chassis->Movement.Vx * cos(angle) - chassis->Movement.Vy * sin(angle));
	
	chassis->Motors.motor[0].Data.Target = (-1) * rotated_vx +   1  * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[1].Data.Target =   1  * rotated_vx +   1  * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[2].Data.Target =   1  * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[3].Data.Target = (-1) * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;	
	
	for(int i=0;i<4;i++)  //< ������̵��
	{   
		if( chassis->Motors.motor[i].Data.Target >  7800)   chassis->Motors.motor[i].Data.Target =  7800;
		if( chassis->Motors.motor[i].Data.Target < -7800)   chassis->Motors.motor[i].Data.Target = -7800;
		chassis->Motors.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors.RunPID + i), chassis->Motors.motor[i].Data.Target, chassis->Motors.motor[i].Data.SpeedRPM);
		MotorFillData(&chassis->Motors.motor[i], chassis->Motors.motor[i].Data.Output);
	}
	 
}


/**
  * @brief  ���ֵ��̷��͵����������
  */
void MecanumChassisOutputControl(MecanumChassis* chassis, RC_Ctrl rcCtrl)
{
	if(rcCtrl.isOnline == 1) 
	{
		MotorCanOutput(can2, 0x1ff);  
		MotorCanOutput(can2, 0x200);		
	}
	else 
	{ //< ���ջ����ߣ�������Ϊ0
		MotorFillData(&chassis->Motors.motor[0],0);
		MotorFillData(&chassis->Motors.motor[1],0);
		MotorFillData(&chassis->Motors.motor[2],0);
		MotorFillData(&chassis->Motors.motor[3],0);
		MotorCanOutput(can2, 0x1ff);
		MotorCanOutput(can2, 0x200);
	}
}
	





