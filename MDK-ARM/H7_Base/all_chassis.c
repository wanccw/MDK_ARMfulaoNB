#include "all_chassis.h"
#include "mecanum_chassis.h"

#define AtR 0.0174532f	              //<  3.1415 /180 角度制 转化为弧度制	

/**
  * @brief  全向轮底盘逆运动学解算，Inverse Kinematics ,根据chassis结构体中的movement结构体解算转速。
  * @notec  处理过后将未进行功率控制的电流参数填写到Motor结构体中。 
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
	
	for(int i=0;i<4;i++)  //< 计算底盘电机
	{   
		if( chassis->Motors.motor[i].Data.Target >  7800)   chassis->Motors.motor[i].Data.Target =  7800;
		if( chassis->Motors.motor[i].Data.Target < -7800)   chassis->Motors.motor[i].Data.Target = -7800;
		chassis->Motors.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors.RunPID + i), chassis->Motors.motor[i].Data.Target, chassis->Motors.motor[i].Data.SpeedRPM);
		MotorFillData(&chassis->Motors.motor[i], chassis->Motors.motor[i].Data.Output);
	}
	 
}


/**
  * @brief  全向轮底盘二轮驱动逆运动学解算，Inverse Kinematics ,根据chassis结构体中的movement结构体解算转速。
  * @notec  处理过后将未进行功率控制的电流参数填写到Motor结构体中。 
  *  
  * 轮子45°运动时，运动解算时的相对摩擦抵消速度会产生不必要的损耗，把功率累加在两个轮子上可能会跑得更快？？？？？？？
  * 疑问 ： 跑直线时，前后两个不需要转动的轮子要不要锁死，不锁死能节省多少功率？不锁死时侧滑程度会不会很大？
  */
void ALLChassisSetSpeedTwo(MecanumChassis* chassis, float canAngle)
{
	float angle = canAngle * AtR;   //正常情况下输入canAngle为0.0f
	float rotated_vy = (chassis->Movement.Vx * sin(angle) + chassis->Movement.Vy * cos(angle));
	float rotated_vx = (chassis->Movement.Vx * cos(angle) - chassis->Movement.Vy * sin(angle));
	
	chassis->Motors.motor[0].Data.Target =   1  * rotated_vx +  1 * chassis->Movement.Vomega;
	chassis->Motors.motor[1].Data.Target = (-1) * rotated_vy +  1 * chassis->Movement.Vomega;
	chassis->Motors.motor[2].Data.Target = (-1) * rotated_vx +  1 * chassis->Movement.Vomega;
	chassis->Motors.motor[3].Data.Target =   1  * rotated_vy +  1 * chassis->Movement.Vomega;	
	
	for(int i=0;i<4;i++)  //< 计算底盘电机
	{   
		if( chassis->Motors.motor[i].Data.Target >  7800)   chassis->Motors.motor[i].Data.Target =  7800;
		if( chassis->Motors.motor[i].Data.Target < -7800)   chassis->Motors.motor[i].Data.Target = -7800;
		chassis->Motors.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors.RunPID + i), chassis->Motors.motor[i].Data.Target, chassis->Motors.motor[i].Data.SpeedRPM);
		MotorFillData(&chassis->Motors.motor[i], chassis->Motors.motor[i].Data.Output);
	}
	 
}