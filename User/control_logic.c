#include "hardware_config.h"
#include "swerve_chassis.h"
#include "control_logic.h"
#include "mecanum_chassis.h"
#include "mpu6050.h"
#include "motor.h"
#include "dr16.h"
#include "load.h" 
#include "pid.h"
#include "holder.h"
#include "brain.h"
#include "vision.h"
#include "stdio.h"
#include "Supercap.h"
#include "Gyro.h"
#include "interaction.h"
#include "check.h"
#include "usart.h"
float target_angle = 0;
float yawTargetAngle = 0;
int16_t yawOutput = 0;
int16_t ThisSecond =0;
int16_t ano_tc_data[8];
int brain_second_cnt=0;
uint8_t XinTiaoBao_cut=0;
uint8_t update1; 
FPS tim14_FPS={
.Receiver_cnt= 0,
.Referee_cnt = 0,
.Vision_cnt = 0,
.Holder_cnt = 0
};
MpuDebug mpudebug =
{
	.mpuDebugTime=0,
	.DebugBeginFlag=0,
	.DebugFinishFlag=0 
//	.MpuAngle1=0,
//  .CanAngle1=0,
//  .MpuAngle2=0,
//  .CanAngle2=0,
//	.MpuAngle3=0,
//  .CanAngle3=0
};
//static uint16_t Reset_Count = 0;
uint16_t Reset_Count = 0;
float sens_pitch_change = 1;
int asd=0;
int time_cut =0;
uint8_t mode =0;
//< TIM14的触发频率在CubeMX中被配置为1000Hz

/**
  * @brief  定时器中断回调
  */
void TIM14_Task(void)
{
	tim14.ClockTime++;
	tim14.HolderTime++;
	tim14.ErrorTime++;
	if(rc_Ctrl.isOnline == 1)
	  tim14.UI_Time++;
	else
		tim14.UI_Time=0;
	Steering_Engine_Control(&rc_Ctrl,&Shoot);
	asd++;
	if(tim14.ClockTime<500)      //tim14.ClockTime>0&&
	{
	 HolderReset(&Holder);
	//	asd++;
	 if(abs(Holder.Motors6020.motor[1].Data.Angle)<=5.0f)
	 {
		 
			Reset_Count++;
			if(Reset_Count>300) //保持正确的角度300ms以上
			{
				Holder.Reset_OK_Flag = 1; //复位成功
				Reset_Count = 300;  
			}	
	 }						
	 else //否则 复位失败
	 { //
		  Holder.Reset_OK_Flag = 0;
		}
	 	if(Holder.Reset_OK_Flag ==1)
		Holder.ESC_Reset_flag = 0; 
	  else if(Holder.Reset_OK_Flag ==0)
		Holder.ESC_Reset_flag = 1;
		if(Holder.ESC_Reset_flag == 1) //如果电机被置位了 重置 clock_time 和 Holder.Yaw_ESC_Reset_flag
		{
			Holder.ESC_Reset_flag = 0;
			time_cut++;
			tim14.ClockTime=0;
		}
		
	 }
	
	RobotOnlineState(&check_robot_state, &referee2022);
	 
  if(tim14.HolderTime>15000){
	tim14.HolderTime = 15000;
	}
	if(tim14.ErrorTime>20000){
	tim14.ErrorTime = 20000;
	}
	if(check_robot_state.usart_state.Check_receiver>30)
		rc_Ctrl.isOnline = 0;
	else 
		rc_Ctrl.isOnline = 1;
	

  update(tim14.UI_Time, referee2022.game_robot_status.robot_id, each_state.update);                                                      //??3?±èèü????UIé??-μ?í???￡¨?1??ê?1yê?·??éó?￡
	referee_draw_supercap_data(tim14.UI_Time, referee2022.game_robot_status.robot_id, super_cap.cap_state.Supercap_Mode, super_cap.cap_state.Supercap_Charge, super_cap.cap_state.Voltage);  //??ê?3???μ?èYμ?á?,3?μ?￡??a1?
	referee_draw_NUC_data(tim14.UI_Time , referee2022.game_robot_status.robot_id, 1, Vision_Info.Hit_Mode, Brain.BrainCore[Brain.FrameCoreID].BrainMode,Shoot.BulletCap_Status_Flag);                                  //×??é?￡ê?μ?UI
	referee_draw_chassis_data(tim14.UI_Time , referee2022.game_robot_status.robot_id, ChassisSwerve.Chassis_State, swerveChassis.SuperPowerMode, Shoot.SuperHeatMode,Holder.Pitch.Can_Angle);                          //×?Dyμ?ê±oò3????2è|￡?????×′ì?????óDí?D?￡?3?1|?ê￡?3?èèá?
//	referee_draw_Load_data(tim14.UI_Time , referee2022.game_robot_status.robot_id, Shoot.BulletCap_Status_Flag);                               //μˉ2??a1?
	referee_draw_shoot_data(tim14.UI_Time , referee2022.game_robot_status.robot_id, each_state.mode, Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle*955/63.99+955,Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle*400/63.99+400);
	draw_robotdata(tim14.UI_Time , referee2022.game_robot_status.robot_id, each_state.level, each_state.state, each_state.danger, super_cap.cap_state.Voltage);      //??μ?Dí??ê?μ?èYμ?á?￡??1?é?ó????


	if(tim14.UI_Time%300==0){
		draw_interaction(referee2022.game_robot_status.robot_id, swerveChassis.Motors3508.motor[0].Data.TorqueCurrent);     //?ú?÷è????￥
	}
	
	if(tim14.UI_Time%600==0)
		draw_client_map_information(referee2022.game_robot_status.robot_id,5,5);                                         //D?μ?í???ê?μ?·?????￡???o?à×′?￡?
	
	if(Holder.Pitch.MPU6050_Angle_Change!=0)
	sens_pitch_change=abs(Holder.Pitch.Can_Angle_Change) /abs(Holder.Pitch.MPU6050_Angle_Change);
  MPU_Get_Data(&Holder);
	
	
	if(tim14.ClockTime%1 == 0)
	{
		if(Holder.ESC_Reset_flag == 0&&tim14.ClockTime>500)
		{
			Friction_Shoot_Heat_Control_Deal(&rc_Ctrl);
	  	Friction_Load_Fire_Control(&Shoot, &Booster,&Vision_Info);
		  HolderGetRemoteData(&Holder, &rc_Ctrl);
		}
		if(rc_Ctrl.isOnline == 1)
		{
			MotorCanOutput(can1, 0x200);
  		MotorCanOutput(can1, 0x1ff);
			SupercapControl(can1, &rc_Ctrl, &super_cap);		
		}
		else
		{
			MotorFillData(&Holder.Motors6020.motor[0],0);
			MotorFillData(&Holder.Motors6020.motor[1],0);
			MotorFillData(&Shoot.Motors3508.motor[0],0);
			MotorFillData(&Shoot.Motors3508.motor[1],0);
			MotorFillData(&Shoot.Motors2006.motor[0],0);
			MotorCanOutput(can1, 0x200);
			MotorCanOutput(can1, 0x1ff);
		}
	}
	
	if(tim14.ClockTime%5 == 0)   //< 发送频率被降低为200Hz
	{
	 if(Holder.ESC_Reset_flag == 0&&tim14.ClockTime>500)
	 {
		Chassis_Mode_Control(&swerveChassis, &rc_Ctrl, &Holder, &ChassisSwerve,&super_cap);
	 }
	 
		if(rc_Ctrl.isOnline == 1)
		{
			MotorCanOutput(can2, 0x1ff);
			MotorCanOutput(can2, 0x2ff);
			MotorCanOutput(can2, 0x200);	
		}
		else 
		{
			MotorFillData(&swerveChassis.Motors6020.motor[0],0);
			MotorFillData(&swerveChassis.Motors6020.motor[1],0);
			MotorFillData(&swerveChassis.Motors6020.motor[2],0);
			MotorFillData(&swerveChassis.Motors6020.motor[3],0);
			MotorFillData(&swerveChassis.Motors3508.motor[0],0);
			MotorFillData(&swerveChassis.Motors3508.motor[1],0);
			MotorFillData(&swerveChassis.Motors3508.motor[2],0);
			MotorFillData(&swerveChassis.Motors3508.motor[3],0);
			MotorCanOutput(can2, 0x1ff);
			MotorCanOutput(can2, 0x2ff);
			MotorCanOutput(can2, 0x200);
		}

	}
	if(tim14.ClockTime%2 == 0)   //< 发送频率被降低为500Hz
	{
	//	Brain_RobotToBrainTime(Holder.Yaw.MPU6050_Angle);   //一秒发送200次四元数及时间给上位机
	}		
	  if((tim14.ClockTime%1==0)&&(Vision_Info.RobotModeFlag==1))
		{
    Brain_RobotModeControl(&Vision_Info,&rc_Ctrl);
    Vision_Info.RobotModeFlag=0;
		}			
	if(tim14.ClockTime%100 == 0)   //两秒发一次心跳包
	{
//		XinTiaoBao_cut++;
//		if(XinTiaoBao_cut>=2)
//		{
//		XinTiaoBao_cut=0;
	  Brain_RobotToBrainQuest_XinTiaoBao(1);
//		}
	}
	if(tim14.ClockTime%10==0)	
//  	Brain_RobotToBrainQuest(AUTOMATICHIT, 1, 16);
	if(tim14.ClockTime%1000 == 0)
	{
		ThisSecond=0;
		tim14_FPS.Receiver_cnt=tim14_FPS.Receiver_FPS;
		tim14_FPS.Referee_cnt = tim14_FPS.Referee_FPS;
		tim14_FPS.Vision_cnt = tim14_FPS.Vision_FPS;
		tim14_FPS.Holder_cnt = tim14_FPS.Holder_FPS;
		tim14_FPS.Receiver_FPS = 0;
		tim14_FPS.Referee_FPS  = 0;
		tim14_FPS.Vision_FPS   = 0;
		tim14_FPS.Holder_FPS   = 0;
		Holder.Pitch.Can_Angle_Change = Holder.Pitch.Last_Can_Angle - Holder.Pitch.Can_Angle;
		Holder.Pitch.MPU6050_Angle_Change = Holder.Pitch.Last_MPU6050_Angle - Holder.Pitch.MPU6050_Angle;
		Holder.Pitch.Last_Can_Angle = Holder.Pitch.Can_Angle;
		Holder.Pitch.Last_MPU6050_Angle = Holder.Pitch.MPU6050_Angle;
	}
	if(tim14.ClockTime%10==0)
	{
		printf("%f,%f,%f\r\n", mpuAngle2.pitch, tim14_FPS.Receiver_FPS, tim14_FPS.Referee_FPS);
//		printf("%f,%f\r\n",(float)referee2022.power_heat_data.chassis_power ,(float)referee2022.power_heat_data.chassis_power_buffer );
//		printf("%f,%f,%f,%f,%f,%f\r\n",Brain.BrainCore[1].CoreInstruction.YawDeflectionAngle, Holder.Yaw.Target_Angle, Holder.Yaw.MPU6050_Angle, Brain.BrainCore[1].CoreInstruction.PitchDeflectionAngle,Holder.Pitch.Target_Angle,Holder.Pitch.MPU6050_Angle);
//	  printf("%f,%f,%f,%f\r\n", Brain.BrainCore[1].CoreInstruction.PitchDeflectionAngle,Holder.Pitch.Target_Angle,Holder.Pitch.MPU6050_Angle,Brain_pitch_add);
	}
	
	if((tim14.ClockTime>1000)&&(mpudebug.DebugFinishFlag==0)&&(mpudebug.DebugBeginFlag==1))
	Mpu6050_senceDebug(&mpudebug,&gyro_data2);
	
}


/**
  * @brief  CAN1接收中断回调
  */
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer)); 
  Supercap_rxCallBack(can1, (*rxBuffer) ,&super_cap);	
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can2, (*rxBuffer)); 	
	return 0;
}

/**
  * @brief  校准陀螺仪角度
  */
uint8_t Mpu6050_senceDebug(MpuDebug* mpuDebug,gyro_data_t* gyro_data)
{
	mpuDebug->mpuDebugTime++;
	if(mpuDebug->mpuDebugTime==200)
	{
		mpuDebug->Yaw.MpuAngle1=Holder.Yaw .MPU6050_Angle;
		mpuDebug->Yaw.CanAngle1=Holder.Yaw.Can_Angle;
		mpuDebug->Pitch.MpuAngle1=Holder.Pitch .MPU6050_Angle;
		mpuDebug->Pitch.CanAngle1=Holder.Pitch.Can_Angle;
	}
	if(mpuDebug->mpuDebugTime==400)
	{
		mpuDebug->Yaw.MpuAngle2=Holder.Yaw .MPU6050_Angle;
		mpuDebug->Yaw.CanAngle2=Holder.Yaw.Can_Angle;
		mpuDebug->Pitch.MpuAngle2=Holder.Pitch .MPU6050_Angle;
		mpuDebug->Pitch.CanAngle2=Holder.Pitch.Can_Angle;
	}
	if(mpuDebug->mpuDebugTime==600)
	{
		mpuDebug->Yaw.MpuAngle3=Holder.Yaw .MPU6050_Angle;
		mpuDebug->Yaw.CanAngle3=Holder.Yaw.Can_Angle;
		mpuDebug->Pitch.MpuAngle3=Holder.Pitch .MPU6050_Angle;
		mpuDebug->Pitch.CanAngle3=Holder.Pitch.Can_Angle;
		mpuDebug->Yaw.MpuAngleChange=((mpuDebug->Yaw.MpuAngle3-mpuDebug->Yaw.MpuAngle2)+(mpuDebug->Yaw.MpuAngle2-mpuDebug->Yaw.MpuAngle1))*0.5;
		mpuDebug->Yaw.CanAngleChange=((mpuDebug->Yaw.CanAngle3-mpuDebug->Yaw.CanAngle2)+(mpuDebug->Yaw.CanAngle2-mpuDebug->Yaw.CanAngle1))*0.5;
		mpuDebug->Pitch.MpuAngleChange=((mpuDebug->Pitch.MpuAngle3-mpuDebug->Pitch.MpuAngle2)+(mpuDebug->Pitch.MpuAngle2-mpuDebug->Pitch.MpuAngle1))*0.5;
		mpuDebug->Pitch.CanAngleChange=((mpuDebug->Pitch.CanAngle3-mpuDebug->Pitch.CanAngle2)+(mpuDebug->Pitch.CanAngle2-mpuDebug->Pitch.CanAngle1))*0.5;
    mpuDebug->Yaw.SenceBili=abs(mpuDebug->Yaw.CanAngleChange)/abs(mpuDebug->Yaw.MpuAngleChange);
		mpuDebug->Pitch.SenceBili=abs(mpuDebug->Pitch.CanAngleChange)/abs(mpuDebug->Pitch.MpuAngleChange);
//		if((mpuDebug->Yaw.SenceBili>=2)||(mpuDebug->Yaw.SenceBili<=0))
//			mpuDebug->Yaw.SenceBili=1;
//		if((mpuDebug->Pitch.SenceBili>=2)||(mpuDebug->Pitch.SenceBili<=0))
//			mpuDebug->Pitch.SenceBili=1;
//		gyro_data->sens_yaw=gyro_data->sens_yaw*(mpuDebug->Yaw.SenceBili);
//		gyro_data->sens_pitch=gyro_data->sens_pitch*(mpuDebug->Pitch.SenceBili);
	}
	if(mpuDebug->mpuDebugTime>600)
	{
		mpuDebug->mpuDebugTime=600;
	  mpuDebug->DebugFinishFlag=1;
	}
}
