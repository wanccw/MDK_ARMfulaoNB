#include "holder.h"
#include "hardware_config.h"
#include "control_logic.h"
#include "Gyro.h"
#include "brain.h"
#include "filter.h"
#include "check.h"

/**
  * @brief 云台初始化 
  */
    void HolderInit(Holder_t* holder, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object pitch_angle_pid , BasePID_Object pitch_speed_pid, BasePID_Object yaw_reset_pid,BasePID_Object pitch_reset_pid,CanNumber canx)
    {
      MotorInit(&holder->Motors6020.motor[1], 8127 , Motor6020, canx, 0x206);
      MotorInit(&holder->Motors6020.motor[0], 2844 , Motor6020, canx, 0x205);   //5206
    	BasePID_Init(&holder->Motors6020.turnPID[0], yaw_angle_pid.Kp, yaw_angle_pid.Ki, yaw_angle_pid.Kd, yaw_angle_pid.KiPartDetachment);
			BasePID_Init(&holder->Motors6020.turnPID[1], yaw_speed_pid.Kp, yaw_speed_pid.Ki, yaw_speed_pid.Kd, yaw_speed_pid.KiPartDetachment);
      BasePID_Init(&holder->Motors6020.turnPID[2], pitch_angle_pid.Kp, pitch_angle_pid.Ki, pitch_angle_pid.Kd, pitch_angle_pid.KiPartDetachment);
      BasePID_Init(&holder->Motors6020.turnPID[3], pitch_speed_pid.Kp, pitch_speed_pid.Ki, pitch_speed_pid.Kd, pitch_speed_pid.KiPartDetachment);
			BasePID_Init(&holder->Motors6020.turnPID[4], yaw_reset_pid.Kp, yaw_reset_pid.Ki, yaw_reset_pid.Kd, yaw_reset_pid.KiPartDetachment);
			BasePID_Init(&holder->Motors6020.turnPID[5], pitch_reset_pid.Kp, pitch_reset_pid.Ki, pitch_reset_pid.Kd, pitch_reset_pid.KiPartDetachment);
      holder->Pitch.Sensitivity = -0.0012f;  //-0.0015f
      holder->Yaw.Sensitivity = 0.0010f;      //0.003f
			holder->cruise = 0.25f;
			holder->Pitch.MouseSensitivity=-0.0045f;
			holder->Yaw.MouseSensitivity=0.02f;
      }
		void HolderReset(Holder_t* holder)
		{
			 holder->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 4) , 0 , holder->Motors6020.motor[0].Data.Angle,holder->Motors6020.motor[0].Data.SpeedRPM);
		   holder->Motors6020.motor[1].Data.Output = BasePID_AngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 5), 0 , holder->Motors6020.motor[1].Data.Angle,holder->Motors6020.motor[1].Data.SpeedRPM);
		   for(int i = 0 ; i < 2 ; i ++)
   		{
   		 if(holder->Motors6020.motor[i].Data.Output>30000)  holder->Motors6020.motor[i].Data.Output = 30000;
   		 if(holder->Motors6020.motor[i].Data.Output<-30000)  holder->Motors6020.motor[i].Data.Output = -30000;
   		}
   	  MotorFillData(&holder->Motors6020.motor[1], holder->Motors6020.motor[1].Data.Output);
    	MotorFillData(&holder->Motors6020.motor[0], holder->Motors6020.motor[0].Data.Output);
		}
    
		
		int zhen =0;
		float up_limit = -45.0f;
		float down_limit = 23.0f;
		float Brain_yaw_add_show=0;
		float Brain_pitch_add_show=0;
		float YawDeflectionAngle_show=0;
		float PitchDeflectionAngle_show=0;
		uint16_t ShootWaitTime = 0;
		uint8_t ShootWaitTimeBegin =0;
/**
  * @brief 遥控器数值控制云台
  */	
    void HolderGetRemoteData(Holder_t* holder, RC_Ctrl* rc_ctrl) 
    {
			tim14_FPS.Holder_FPS++;
				if(Holder.Direction_Flag==1)
				((&holder->Motors6020.motor[0])->Param).EcdOffset = 777;
			else 
				((&holder->Motors6020.motor[0])->Param).EcdOffset = 2844;
			if(tim14.ClockTime>500)
			{
//				 if(Vision_Info.Hit_Mode==0)
//				 {
//							Brain_yaw_add =0;
//							Brain_pitch_add = 0;
//							Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle =0;
//				      Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle =0;
//					    LastYawDeflectionAngle=0;
//					    LastPitchDeflectionAngle=0;
//				 }
//				 if((Brain.FrameType ==1)&&((Vision_Info.Hit_Mode==2)||(Vision_Info.Hit_Mode==1)))
//				 {
//			      if( LastPitchDeflectionAngle == Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle)  //判断视觉是否掉线，视觉标志位转变间隔1s多，过长，不可用
//				       PitchDeflectionAngleCut++;
//			      else
//				       PitchDeflectionAngleCut=0;
//			      if(PitchDeflectionAngleCut>=200)  //10*7
//			        {
//		           Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle =0;
//				       Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle =0;
//			         PitchDeflectionAngleCut=200;
//			         }
						
//							Brain_add_cut +=0.5;
//							if(Brain_add_cut>=1)
//								Brain_add_cut=1; 
//					    Brain_yaw_add = (Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle*Brain_add_cut + LastYawDeflectionAngle*(1-Brain_add_cut)) * YawSence ;
//							Brain_pitch_add =(Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle*Brain_add_cut + LastPitchDeflectionAngle*(1-Brain_add_cut)) * PitchSence;
							
//							if(Brain_yaw_add>10.0f)
//								Brain_yaw_add=10.0f;
//							else if(Brain_yaw_add<-10.0f)
//								Brain_yaw_add=-10.0f;
//							if(Brain_pitch_add>10.0f)
//								Brain_pitch_add=10.0f;
//							else if(Brain_pitch_add<-10.0f)
//								Brain_pitch_add=-10.0f;
							
//							Brain_yaw_add_show=Brain_yaw_add;
//							Brain_pitch_add_show=Brain_pitch_add;
//							YawDeflectionAngle_show=Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle;
//							PitchDeflectionAngle_show= Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle;
							
//							if((Vision_Info.buchang == 0)&&(Brain_yaw_add!=0)&&(Brain_pitch_add!=0))
//					  {
////							Holder.Yaw.Target_Angle += Brain_yaw_add;
////							Holder.Pitch.Target_Angle += Brain_pitch_add;
//						Holder.Yaw.Target_Angle = Brain_yaw_add+ Holder.Yaw.MPU6050_Angle ;
//		        Holder.Pitch.Target_Angle = Brain_pitch_add + Holder.Pitch.MPU6050_Angle ;
//							
////						Holder.Yaw.Target_Angle = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle * YawSence + Holder.Yaw.MPU6050_Angle ;
////		        Holder.Pitch.Target_Angle = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle * PitchSence + Holder.Pitch.MPU6050_Angle ;
//						}
//						if(Vision_Info.buchang == 1)
//						{
//            Holder.Pitch.Target_Angle = Brain_pitch_add + Holder.Pitch.MPU6050_Angle ;	
//						}
//						LastYawDeflectionAngle = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.YawDeflectionAngle;
//						LastPitchDeflectionAngle = Brain.BrainCore[Brain.FrameCoreID].CoreInstruction.PitchDeflectionAngle;
//				 }
//      }
      if(check_robot_state.usart_state.Check_vision!=0)
			holder->Pitch.Target_Angle += ((rc_ctrl->rc.ch3 -1024)* holder->Pitch.Sensitivity-rc_ctrl->mouse.y*holder->Pitch.MouseSensitivity);
			}
		if(check_robot_state.usart_state.Check_vision!=0)	
		{
			if((rc_ctrl->Cruise_Mode == 1)&&(tim14.ClockTime>500))
      {
			    holder->Yaw.Target_Angle += ((rc_ctrl->rc.ch2 -1024)* holder->Yaw.Sensitivity+rc_ctrl->mouse.x*holder->Yaw.MouseSensitivity +holder->cruise);			
			}
			if((rc_ctrl->Cruise_Mode == 0)&&(tim14.ClockTime>500))
      {
			    holder->Yaw.Target_Angle += ((rc_ctrl->rc.ch2 -1024)* holder->Yaw.Sensitivity+rc_ctrl->mouse.x*holder->Yaw.Sensitivity) ;
			}
		}
      holder->Yaw.Can_Angle = holder->Motors6020.motor[0].Data.Angle;
			holder->Pitch.Can_Angle = holder->Motors6020.motor[1].Data.Angle;
			holder->Yaw.Can_Angle_speed  =  holder->Motors6020.motor[0].Data.SpeedRPM*6;
			holder->Pitch.Can_Angle_speed  =  holder->Motors6020.motor[1].Data.SpeedRPM;
//			if(holder->Pitch.Target_Angle <= -50.0f)
//    	holder->Pitch.Target_Angle = -50.0f;
//    	else if(Holder.Pitch.Target_Angle >= 28.0f)
//    	Holder.Pitch.Target_Angle = 28.0f;
//			if(tim14.ClockTime%1000==1)
//			{
			up_limit = holder->Pitch.MPU6050_Angle-holder->Motors6020.motor[1].Data.Angle-40.0f;
//			up_limit = LPFilter(up_limit ,&LPF_up);
			down_limit = holder->Pitch.MPU6050_Angle-holder->Motors6020.motor[1].Data.Angle+22.0f;
//			down_limit = LPFilter(down_limit ,&LPF_down);
//			}
			if(holder->Pitch.Target_Angle <= up_limit)
    	holder->Pitch.Target_Angle = up_limit;
    	else if(Holder.Pitch.Target_Angle >= down_limit)
    	Holder.Pitch.Target_Angle = down_limit;
		//holder->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 0) , holder->Yaw.Target_Angle , holder->Yaw.Bmi088_Angle , holder->Yaw.Bmi088_Angle_speed);
    //holder->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 0) , holder->Yaw.Target_Angle , holder->Motors6020.motor[0].Data.Angle , holder->Motors6020.motor[0].Data.AngleSpeed);
    //holder->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 0) , holder->Yaw.Target_Angle , holder->Yaw.MPU6050_Angle , holder->Yaw.MPU6050_Angle_speed1);
      holder->Motors6020.motor[0].Data.Output =BasePID_YawSpeedControl((BasePID_Object*)(holder->Motors6020.turnPID + 1) , BasePID_YawAngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 0) , holder->Yaw.Target_Angle , holder->Yaw.MPU6050_Angle)  ,holder->Yaw.MPU6050_Angle_speed1);
		  holder->Motors6020.motor[1].Data.Output =BasePID_PitchSpeedControl((BasePID_Object*)(holder->Motors6020.turnPID + 3) , BasePID_PitchAngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 2) , holder->Pitch.Target_Angle , holder->Pitch.MPU6050_Angle)  , holder->Pitch.MPU6050_Angle_speed1);
    //holder->Motors6020.motor[1].Data.Output =BasePID_PitchSpeedControl((BasePID_Object*)(holder->Motors6020.turnPID + 3) , BasePID_PitchAngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 2) , holder->Pitch.Target_Angle , holder->Pitch.Can_Angle)  , holder->Pitch.Can_Angle_speed);
   	//holder->Motors6020.motor[1].Data.Output = BasePID_AngleControl((BasePID_Object*)(holder->Motors6020.turnPID + 1),holder->Pitch.Target_Angle,holder->Motors6020.motor[1].Data.Angle,holder->Motors6020.motor[1].Data.SpeedRPM);
			
			holder->Yaw.Angle_error=holder->Yaw.Target_Angle-holder->Yaw.MPU6050_Angle;
			holder->Pitch.Angle_error=holder->Pitch.Target_Angle-holder->Pitch.MPU6050_Angle;
			
//			if((abs(holder->Yaw.Angle_error)<0.3f)&&(abs(holder->Pitch.Angle_error)<0.3f)&&(dafu_shoot_flag==1))
//			{
//				dafu_cut++;
//				if(dafu_cut>=5)
//				{
//					ShootWaitTimeBegin=1;
////					rc_Ctrl.OneShoot +=1;
//					dafu_cut=0;
//					dafu_shoot_flag=0;
//				}
//			}
//			else
//				dafu_cut=0;
//			if(ShootWaitTimeBegin==1)
//			{
//				ShootWaitTime++;
//				if(ShootWaitTime>=140)
//				{
//					rc_Ctrl.OneShoot +=1;
//				  ShootWaitTime =0;
//					ShootWaitTimeBegin=0;
//				}
//			}
      if(dafu_shoot_flag==1)
			{
				ShootWaitTime++;
				if(ShootWaitTime>=157)
				{
					rc_Ctrl.OneShoot +=1;
				  ShootWaitTime =0;
					dafu_shoot_flag=0;
				}
			}
			
			for(int i = 0 ; i < 1 ; i ++)
   		{
   		 if(holder->Motors6020.motor[i].Data.Output>30000)  holder->Motors6020.motor[i].Data.Output = 30000;
   		 if(holder->Motors6020.motor[i].Data.Output<-30000)  holder->Motors6020.motor[i].Data.Output = -30000;
   		}
   	MotorFillData(&holder->Motors6020.motor[1], holder->Motors6020.motor[1].Data.Output);
   	MotorFillData(&holder->Motors6020.motor[0], holder->Motors6020.motor[0].Data.Output);
 }