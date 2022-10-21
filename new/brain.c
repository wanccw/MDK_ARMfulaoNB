#include "brain.h"
#include "vision.h"
#include "hardware_config.h"
#include "Gyro.h"
#include "control_logic.h"
#include "check.h"
#include "filter.h"
#include "mpu6050.h"
uint8_t Brain_recData[Brain_rxBufferLengh]__attribute__((at(0x24018000)));

UART_RxBuffer uart2_buffer;

uint8_t RobotToBrainQuestBuffer[5] __attribute__((at(0x24002020)));
uint8_t RobotToBrainCMDBuffer[3]__attribute__((at(0x24002040)));                   //命令数据
uint8_t RobotToBrainLogBuffer[20]__attribute__((at(0x24002060)));                  //日志数据
uint8_t RobotToBrainTimeBuffer[21]__attribute__((at(0x24002160)));
uint8_t RobotToBrainQuestBuffer_XinTiaoBao[3] __attribute__((at(0x24002200)));     //请求数据
uint8_t RobotToBrainQuestBuffer_WorkingModel[5] __attribute__((at(0x24002210)));   //请求数据
uint8_t RobotToBrainQuestBuffer_Velocity[5] __attribute__((at(0x24002220))); 

float  BrainTenYawAngle[10];
float  BrainTenPitchAngle[10];

CubotBrain_t Brain;
uint8_t ShootModeToBrain = 0;
Forecast forecast={
	.forecast_cut=0,
  .forecast_flag=0,
  .yaw_sum=0,
  .pitch_sum=0,
  .yaw_average=0,
  .pitch_average=0,
  .yaw_error=0,
  .pitch_error=0
};

UART_RxBuffer uart2_buffer={
	.Data = Brain_recData,
	.Size = Brain_rxBufferLengh
};

 float YawSence =1.0f;//0.7f;
 float PitchSence =1.0f;//0.5f;
 float LastYawDeflectionAngle = 0;
 float LastPitchDeflectionAngle = 0;
 int PitchDeflectionAngleCut = 0;
 float Brain_yaw_add = 0;
 float Brain_pitch_add = 0;
 float Brain_add_cut = 0;
 uint16_t dafu_cut=0;
 uint8_t dafu_shoot_flag=0;
;
/**
  * @brief   串口2视觉回调函数 
  * @param[in]  
  */
uint8_t Brain_callback(uint8_t * recBuffer, uint16_t len)
{
  Brain_DataUnpack(&Brain,&Vision_Info,recBuffer);
//	Holder.Yaw.Target_Angle = Brain.BrainCore[1].CoreInstruction.YawDeflectionAngle*0.5f + Holder.Yaw.MPU6050_Angle ;
//	Holder.Pitch.Target_Angle =Brain.BrainCore[1].CoreInstruction.PitchDeflectionAngle *0.5f + Holder.Pitch.Can_Angle ;
	
	return 0;
}

/**
  * @brief  注册内核的回调函数
  */
void Brain_Init(CubotBrain_t* brain, uint8_t index, Brain_CoreCallback callback)
{
	brain->BrainCore[index].CoreCallback = callback;
}


/**
  * @brief  上位机数据解算
  */
void Brain_DataUnpack(CubotBrain_t* brain,Trace* Info_Vision, uint8_t* recBuffer)
{
	if(recBuffer[0] == 0xAA)
	{
		PitchDeflectionAngleCut=0;
//		Brain_yaw_add = 0;
//    Brain_pitch_add = 0;
		
		Brain_add_cut = 0;
		brain->FrameType   = recBuffer[1];
		brain->FrameCoreID = recBuffer[2];
		if((brain->FrameType == BRAIN_TO_ROBOT_CMD) && recBuffer[9] == 0xDD)  //< 解算偏转角
		{
			//< yaw偏转角 在（-63.99 ~ +63.99）范围内
			if((recBuffer[3] >> 6) == 0) 
				brain->BrainCore[brain->FrameCoreID].CoreInstruction.YawDeflectionAngle = ((float)((recBuffer[3]&0x3f)*100 + recBuffer[4])/100);
			else if((recBuffer[3] >> 6) == 1) 
				brain->BrainCore[brain->FrameCoreID].CoreInstruction.YawDeflectionAngle = (-1) * ((float)((recBuffer[3]&0x3f)*100 + recBuffer[4])/100);
			//< pitch偏转角 在（-63.99 ~ +63.99）范围内
			if((recBuffer[5] >> 6) == 0) 
				brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle = ((float)((recBuffer[5]&0x3f)*100 + recBuffer[6])/100);
			else if((recBuffer[5] >> 6) == 1) 
				brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle = (-1) * ((float)((recBuffer[5]&0x3f)*100 + recBuffer[6])/100);
			//< 距离信息 在（0 ~ 12.7）范围内
			brain->BrainCore[brain->FrameCoreID].CoreInstruction.Distance = ((float)(recBuffer[7]))/10;
			brain->BrainCore[brain->FrameCoreID].CoreInstruction.IsFire = ((float)(recBuffer[8]));
			tim14_FPS.Vision_FPS++;
			check_robot_state.usart_state.Check_vision = 0;
			
/*
//			   dafu_cut ++;
//				if(dafu_cut>=100)
//				{
//				 dafu_cut=100;
//				}
//			if((brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle<1)&&(brain->BrainCore[brain->FrameCoreID].CoreInstruction.YawDeflectionAngle<1)&&(Info_Vision->Hit_Mode==1))
//			{	
//			   dafu_cut ++;
//				if(dafu_cut>=10)
//				{
//			   rc_Ctrl.OneShoot +=1;
//				 dafu_cut=0;
//				}
//				if(dafu_cut == 100)
//				{
//					rc_Ctrl.OneShoot +=1;
//  				 dafu_cut=0;
//				}
//			}
//			else 
//				dafu_cut=0;*/

   
	 if(Info_Vision->Hit_Mode==1)
		 {
			 dafu_shoot_flag=1;
		 }

		 if((abs(forecast.yaw_error)>0.01)&&(abs(forecast.yaw_error)<0.15))
		 {
			 if((forecast.yaw_average>=0)&&(forecast.yaw_error>0))
				 forecast.yaw_add=0.1;
			 if((forecast.yaw_average>=0)&&(forecast.yaw_error<0))
				 forecast.yaw_add=-0.1;
			 if((forecast.yaw_average<0)&&(forecast.yaw_error>0))
				 forecast.yaw_add=-0.1;
			 if((forecast.yaw_average<0)&&(forecast.yaw_error>0))
				 forecast.yaw_add=0.1;
		 }
		 else
			 forecast.yaw_add=0;
			 
      Brain_yaw_add = brain->BrainCore[brain->FrameCoreID].CoreInstruction.YawDeflectionAngle+forecast.yaw_add;
			Brain_pitch_add = brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle;
			
//			Brain_yaw_add = LPFilter(brain->BrainCore[brain->FrameCoreID].CoreInstruction.YawDeflectionAngle ,&LPF_yaw_brain);
//			Brain_pitch_add = LPFilter(brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle ,&LPF_pitch_brain);
      if(((Vision_Info.Hit_Mode==2)||(Vision_Info.Hit_Mode==1))&&(Brain_yaw_add!=0)&&(Brain_pitch_add!=0)&&(tim14.ClockTime>=1000))
			{
     		if(Vision_Info.buchang == 0)
				 {
						mpudebug.DebugBeginFlag=1;
//							Holder.Yaw.Target_Angle += Brain_yaw_add;
//							Holder.Pitch.Target_Angle += Brain_pitch_add;
						Holder.Yaw.Target_Angle = Brain_yaw_add+ Holder.Yaw.MPU6050_Angle ;
		        Holder.Pitch.Target_Angle = Brain_pitch_add + Holder.Pitch.MPU6050_Angle ;
//						Holder.Yaw.Target_Angle = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle * YawSence + Holder.Yaw.MPU6050_Angle ;
//		        Holder.Pitch.Target_Angle = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle * PitchSence + Holder.Pitch.MPU6050_Angle ;
			 	}
				else if(Vision_Info.buchang == 1)
					{
            Holder.Pitch.Target_Angle = Brain_pitch_add + Holder.Pitch.MPU6050_Angle ;	
					}			 	

/*		 
//			 LastPitchDeflectionAngle = brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle;
//			 if( LastPitchDeflectionAngle == brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle)
//				 PitchDeflectionAngleCut++;
//			 else
//				 PitchDeflectionAngleCut=0;
//			 if(PitchDeflectionAngleCut>=20)
//			 {
//		    brain->BrainCore[brain->FrameCoreID].CoreInstruction.YawDeflectionAngle =0;
//				brain->BrainCore[brain->FrameCoreID].CoreInstruction.PitchDeflectionAngle =0;
//				PitchDeflectionAngleCut=20;
//			 }
*/					
	if(Info_Vision->Hit_Mode==2)
	{		
		 BrainTenYawAngle[forecast.forecast_cut]=LastYawDeflectionAngle;
		 BrainTenPitchAngle[forecast.forecast_cut]=LastPitchDeflectionAngle;
		 forecast.forecast_cut++;
		 if(forecast.forecast_cut==9)
		 {
			 forecast.forecast_flag=1;
			 forecast.forecast_cut=0;
		 } 
			if(forecast.forecast_flag==1)
			{
				forecast.yaw_sum=0;
				forecast.pitch_sum=0;
				for(int i=0;i<10;i++)
				{
				forecast.yaw_sum+=BrainTenYawAngle[i];
				forecast.pitch_sum+=BrainTenPitchAngle[i];
				forecast.yaw_average=forecast.yaw_sum*0.1;
				forecast.pitch_average=forecast.pitch_sum*0.1;
				forecast.yaw_error=Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle-forecast.yaw_average;
				forecast.pitch_error=Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle-forecast.pitch_average;
				}
			}
			LastYawDeflectionAngle   = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle;
			LastPitchDeflectionAngle = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle;
			}
		}
			if(Info_Vision->Hit_Mode==0)
			{
			 forecast.forecast_flag=0;
			 forecast.forecast_cut=0;
			}				
		}
		else if((brain->FrameType == BRAIN_TO_ROBOT_HINT) && recBuffer[6] == 0xDD)  //< 解算brain状态
		{
			brain->BrainCore[brain->FrameCoreID].BrainMode     = recBuffer[3];
			brain->BrainCore[brain->FrameCoreID].BrainVelocity = recBuffer[4]; 
			brain->BrainCore[brain->FrameCoreID].CoreFlag.Working  = (recBuffer[5] & 0x08) >> 3;	
		  brain->BrainCore[brain->FrameCoreID].CoreFlag.Connect  = (recBuffer[5] & 0x04) >> 2;
		  brain->BrainCore[brain->FrameCoreID].CoreFlag.Open     = (recBuffer[5] & 0x02) >> 1;		
		  brain->BrainCore[brain->FrameCoreID].CoreFlag.Init     = (recBuffer[5] & 0x01);			
			
		}
	}
}



void Brain_RobotModeControl(Trace* Info_Vision,RC_Ctrl* rc_ctrl)
{
//	if(Info_Vision->Hit_Mode==2)
// {
	if(rc_ctrl->ShootNumber == 0)
	{
		Brain_RobotToBrainQuest_WorkingModel(MANUAL, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 1)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTONE, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 2)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTTWO, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 3)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTTHREE, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 4)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTFOUR, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 5)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTFIVE, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 6)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTSENTRY, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 7)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTOUTPOST, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 8)
	{
		Brain_RobotToBrainQuest_WorkingModel(SHOOTBASE, 1, 2);
	}
	if(rc_ctrl->ShootNumber == 9)
	{
//		Brain_RobotToBrainQuest_WorkingModel(AUTOMATICHIT, 1, 2);
		Brain_RobotToBrainQuest_Velocity(1,3,15);
	}
// }
// if(Info_Vision->Hit_Mode==1)     //跳过中间吊射部分
// {
   if(rc_ctrl->ShootNumber == 10)
   {
	  Brain_RobotToBrainQuest_WorkingModel(SHOOTSMALLBUFF, 1, 2);
   }
	 if(rc_ctrl->ShootNumber == 11)
	 {
	 Brain_RobotToBrainQuest_WorkingModel(SHOOTLARGEBUFF, 1, 2);
	 }
// }
}


/**
  * @brief  下位机向上位机请求数据   //旧
  */
void Brain_RobotToBrainQuest(uint8_t mode, uint8_t coreID, uint8_t velocity)
{
	RobotToBrainQuestBuffer[0] = 0xAA;
	RobotToBrainQuestBuffer[1] = (ROBOT_TO_BRAIN_QUEST << 4) | coreID ;
	RobotToBrainQuestBuffer[2] = mode;
	RobotToBrainQuestBuffer[3] = velocity;
	RobotToBrainQuestBuffer[4] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainQuestBuffer, 5);
}


///**
//  * @brief  下位机向上位机发送控制信息    //旧
//  */
//void Brain_RobotToBrainCmd(uint8_t command, uint8_t coreID)
//{
//	RobotToBrainCMDBuffer[0] = 0xAA;
//	RobotToBrainCMDBuffer[1] = (ROBOT_TO_BRAIN_CMD << 4) | coreID ;
//	RobotToBrainCMDBuffer[2] = command;
//	RobotToBrainCMDBuffer[3] = 0xDD;
//	HAL_UART_Transmit_DMA(&huart2, RobotToBrainCMDBuffer, 4);
//}


///**
//  * @brief  下位机向上位机发送日志        //旧
//  */
//void Brain_RobotToBrainLog(char* log_string, uint8_t coreID)
//{
//	RobotToBrainLogBuffer[0] = 0xAA;
//	RobotToBrainLogBuffer[1] = (ROBOT_TO_BRAIN_LOG << 4) | coreID;
//	strcpy((char* )(RobotToBrainLogBuffer+2), log_string);
//	RobotToBrainLogBuffer[sizeof(log_string)+2] = 0xDD;
//	HAL_UART_Transmit_DMA(&huart2, RobotToBrainLogBuffer, sizeof(log_string)+3);
//}


/**
* @brief  下位机向上位机心跳包传送
  */
void Brain_RobotToBrainQuest_XinTiaoBao(uint8_t Type)
{
	RobotToBrainQuestBuffer_XinTiaoBao[0] = 0xAA;
	RobotToBrainQuestBuffer_XinTiaoBao[1] = Type & 0x0F;   //固定为0x01
	RobotToBrainQuestBuffer_XinTiaoBao[2] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainQuestBuffer_XinTiaoBao, 3);
}



/**
  * @brief  下位机向上位机工作模式切换请求
  */
void Brain_RobotToBrainQuest_WorkingModel(uint8_t mode, uint8_t coreID,uint8_t Type)
{
	RobotToBrainQuestBuffer_WorkingModel[0] = 0xAA;
	RobotToBrainQuestBuffer_WorkingModel[1] = Type;        //固定为0x02
	RobotToBrainQuestBuffer_WorkingModel[2] = coreID;      //机器人大脑内核编号，目前固定为0x01
	RobotToBrainQuestBuffer_WorkingModel[3] = mode;
	RobotToBrainQuestBuffer_WorkingModel[4] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainQuestBuffer_WorkingModel, 5);
}



/**
  * @brief  下位机向上位机速度切换请求
  */
void Brain_RobotToBrainQuest_Velocity(uint8_t coreID,uint8_t Type,uint8_t Velocity)
{
	RobotToBrainQuestBuffer_Velocity[0] = 0xAA;
	RobotToBrainQuestBuffer_Velocity[1] = Type;    //固定为0x03
	RobotToBrainQuestBuffer_Velocity[2] = coreID;
	RobotToBrainQuestBuffer_Velocity[3] = Velocity;
	RobotToBrainQuestBuffer_Velocity[4] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainQuestBuffer_Velocity, 5);
}



/**
  * @brief  下位机向上位机发送日志
  */
void Brain_RobotToBrainLog(char* log_string, uint8_t coreID,uint8_t Type)
{
	RobotToBrainLogBuffer[0] = 0xAA;
	RobotToBrainLogBuffer[1] = Type;    //固定为0x04
	RobotToBrainLogBuffer[2] = coreID;
	strcpy((char* )(RobotToBrainLogBuffer+3), log_string);
	RobotToBrainLogBuffer[sizeof(log_string)+3] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainLogBuffer, sizeof(log_string)+4);
}



/**
  * @brief  下位机向上位机发送机器人大脑重启请求
  */
void Brain_RobotToBrainCmd(uint8_t Type)
{
	RobotToBrainCMDBuffer[0] = 0xAA;
	RobotToBrainCMDBuffer[1] = Type ;  //固定为0x05
	RobotToBrainCMDBuffer[2] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainCMDBuffer, 3);
}

/**
  * @brief  下位机向上位机发送计算机重启请求
  */
void Brain_RobotToComputerCmd(uint8_t Type)
{
	RobotToBrainCMDBuffer[0] = 0xAA;
	RobotToBrainCMDBuffer[1] = Type ;  //固定为0x06
	RobotToBrainCMDBuffer[2] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainCMDBuffer, 3);
}

/**
  * @brief  下位机向上位机发送时间戳以及四元数
  */

int16_t tmp0,tmp1,tmp2,tmp3,tmp4,cnt;
float yawyaw;
void Brain_RobotToBrainTime(float yaw)
{
	yawyaw = yaw * 3.1701;
	ThisSecond++;
	tmp0 = (int16_t)(imu_four.Q0 *  30000);
	tmp1 = (int16_t)(imu_four.Q1 *  30000);
	tmp2 = (int16_t)(imu_four.Q2 *  30000);
	tmp3 = (int16_t)(imu_four.Q3 *  30000);

	tmp4 = (int16_t)(yawyaw * 100) % 36000;	 //单圈角度
	cnt = (yawyaw * 100)/360;
	
	if(tmp4 > 18000)	
		tmp4 = tmp4 - 36000; // 转换为0  ->  -180
	else if(tmp4 < -18000)								
		tmp4 = 36000 + tmp4;			   // 为0    ->   180
		
	
	RobotToBrainTimeBuffer[0]  = 0xAA;
	RobotToBrainTimeBuffer[1]  = 0x07;                      //Type ;  //固定为0x07
	RobotToBrainTimeBuffer[2]  = 0x01;                      //coreID;  //目前固定为0x01
	RobotToBrainTimeBuffer[3]  = (ThisSecond >> 8) ;        //索引，int16_t型
	RobotToBrainTimeBuffer[4]  = (ThisSecond &0xff);
	RobotToBrainTimeBuffer[5]  = ( tim14.ClockTime >>24);    //定时器时间，int32_t型
	RobotToBrainTimeBuffer[6]  = ((tim14.ClockTime >>16)&0xff);
	RobotToBrainTimeBuffer[7]  = ((tim14.ClockTime >>8)&0xff);
	RobotToBrainTimeBuffer[8]  = ((tim14.ClockTime &0xff));
	RobotToBrainTimeBuffer[9]  = ShootModeToBrain;                 //0 不开启预测不击打，1 开启预测并击打
	RobotToBrainTimeBuffer[10] = tmp0 & 0xFF;                   //四元数q0，float型
	RobotToBrainTimeBuffer[11] = tmp0 >> 8;
	RobotToBrainTimeBuffer[12] = tmp1 & 0xFF;
	RobotToBrainTimeBuffer[13] = tmp1 >> 8;  
	RobotToBrainTimeBuffer[14] = tmp2 & 0xFF;                   //四元数q1，float型
	RobotToBrainTimeBuffer[15] = tmp2 >> 8;
	RobotToBrainTimeBuffer[16] = tmp3 & 0xFF;
	RobotToBrainTimeBuffer[17] = tmp3 >> 8;  
	RobotToBrainTimeBuffer[18] = tmp4 & 0xFF;
	RobotToBrainTimeBuffer[19] = tmp4 >> 8;  
	RobotToBrainTimeBuffer[20] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainTimeBuffer, 21);
}