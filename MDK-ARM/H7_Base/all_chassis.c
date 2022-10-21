#include "all_chassis.h"
#include "mecanum_chassis.h"

#define AtR 0.0174532f	              //<  3.1415 /180 �Ƕ��� ת��Ϊ������	

/**
  * @brief  ȫ���ֵ������˶�ѧ���㣬Inverse Kinematics ,����chassis�ṹ���е�movement�ṹ�����ת�١�
  * @notec  �������δ���й��ʿ��Ƶĵ���������д��Motor�ṹ���С� 
  */
void ALLChassisSetSpeed(MecanumChassis* chassis, float canAngle)
{
	float angle = canAngle * AtR;
	float rotated_vy = (chassis->Movement.Vx * sin(angle) + chassis->Movement.Vy * cos(angle));
	float rotated_vx = (chassis->Movement.Vx * cos(angle) - chassis->Movement.Vy * sin(angle));
	
	chassis->Motors.motor[0].Data.Target =   1  * rotated_vx +   1  * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[1].Data.Target =   1  * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[2].Data.Target = (-1) * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[3].Data.Target = (-1) * rotated_vx +   1  * rotated_vy + 1 * chassis->Movement.Vomega;	
	
	for(int i=0;i<4;i++)  //< ������̵��
	{   
		if( chassis->Motors.motor[i].Data.Target >  7800)   chassis->Motors.motor[i].Data.Target =  7800;
		if( chassis->Motors.motor[i].Data.Target < -7800)   chassis->Motors.motor[i].Data.Target = -7800;
		chassis->Motors.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors.RunPID + i), chassis->Motors.motor[i].Data.Target, chassis->Motors.motor[i].Data.SpeedRPM);
		MotorFillData(&chassis->Motors.motor[i], chassis->Motors.motor[i].Data.Output);
	}
	 
}


/**
  * @brief  ȫ���ֵ��̶����������˶�ѧ���㣬Inverse Kinematics ,����chassis�ṹ���е�movement�ṹ�����ת�١�
  * @notec  �������δ���й��ʿ��Ƶĵ���������д��Motor�ṹ���С� 
  *  
  * ����45���˶�ʱ���˶�����ʱ�����Ħ�������ٶȻ��������Ҫ����ģ��ѹ����ۼ������������Ͽ��ܻ��ܵø��죿������������
  * ���� �� ��ֱ��ʱ��ǰ����������Ҫת��������Ҫ��Ҫ�������������ܽ�ʡ���ٹ��ʣ�������ʱ�໬�̶Ȼ᲻��ܴ�
  */
void ALLChassisSetSpeedTwo(MecanumChassis* chassis, float canAngle)
{
	float angle = canAngle * AtR;   //�������������canAngleΪ0.0f
	float rotated_vy = (chassis->Movement.Vx * sin(angle) + chassis->Movement.Vy * cos(angle));
	float rotated_vx = (chassis->Movement.Vx * cos(angle) - chassis->Movement.Vy * sin(angle));
	
	chassis->Motors.motor[0].Data.Target =   1  * rotated_vx +  1 * chassis->Movement.Vomega;
	chassis->Motors.motor[1].Data.Target = (-1) * rotated_vy +  1 * chassis->Movement.Vomega;
	chassis->Motors.motor[2].Data.Target = (-1) * rotated_vx +  1 * chassis->Movement.Vomega;
	chassis->Motors.motor[3].Data.Target =   1  * rotated_vy +  1 * chassis->Movement.Vomega;	
	
	for(int i=0;i<4;i++)  //< ������̵��
	{   
		if( chassis->Motors.motor[i].Data.Target >  7800)   chassis->Motors.motor[i].Data.Target =  7800;
		if( chassis->Motors.motor[i].Data.Target < -7800)   chassis->Motors.motor[i].Data.Target = -7800;
		chassis->Motors.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors.RunPID + i), chassis->Motors.motor[i].Data.Target, chassis->Motors.motor[i].Data.SpeedRPM);
		MotorFillData(&chassis->Motors.motor[i], chassis->Motors.motor[i].Data.Output);
	}
	 
}