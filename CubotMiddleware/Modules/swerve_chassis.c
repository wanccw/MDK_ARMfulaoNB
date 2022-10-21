#include "swerve_chassis.h"
#include "math.h"
#include "hardware_config.h"
#include "referee.h"
#define myABS(x) ( (x)>0?(x):-(x) )
#define AtR 0.0174532f	              //<  3.1415 /180 �Ƕ��� ת��Ϊ������	
int16_t Speed_Limit =4000;//4000 ;
int32_t Output_6020 =0;
int8_t rush_cut =0;
int8_t rush_flag=0;
uint8_t fly_flag=0;
int16_t fly_angle[4]={-5091,5200,5200,-5091};
int16_t fly_angle2[4]={150,9000,-10000,150};
/**
  * @brief  ������������������ϵ�µķ��������������ģ�ͽǶ�
  */
void VectorSolve(int16_t vx, int16_t vy, float* vectorAngle, int16_t* vectorModule, uint8_t id)
{
	float vxDivideVy;
	float angle;
	static float lastAngle[4];
	int16_t module;
	
	//< ������ջ�����������ƽ���ٶȺ������ĽǶȺ�ģֵ
	if((vx >100 || vx < -100) || (vy >100 || vy < -100))
	{
		vxDivideVy = (float)vx / (float)vy;
		angle =  57.29f * atan(vxDivideVy);
		module  =  sqrt(vx * vx + vy * vy);
		
		//< �����Ǻ���ϵ�µĽǶ�ֵת��Ϊ0-180���0-(-180��)
		if(vx >= 0 && vy < 0)
			angle = 180 + angle;
		else if(vx < 0 && vy < 0 )
			angle = -180 + angle;	
		if(id < 4)
			lastAngle[id] = angle;
	}
	else
	{
		angle = lastAngle[id];
		module  =  0;
	}
	if(id < 4)
		*vectorAngle  = angle;
	*vectorModule = module;
}


/**
  * @brief  �������������ض������λ��ת��Ϊ+-180��ĽǶ�
  */
float EcdtoAngle(int16_t offset, int16_t ecd)
{
	float angle;
	if(offset < 4096)
	{
		if(ecd > offset + 4096)
			ecd = ecd - 8192;
	}
	else
	{
		if(ecd < offset - 4096)
			ecd = ecd + 8192;
	}
	angle = K_ECD_TO_ANGLE * (ecd - offset);
	return angle;
}



/**
  * @brief  ���ֵ��̳�ʼ��
  */
void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, CanNumber canx)
{
	
	MotorInit(&chassis->Motors3508.motor[0], 0 , Motor3508, canx, 0x201);
	MotorInit(&chassis->Motors3508.motor[1], 0 , Motor3508, canx, 0x202);
	MotorInit(&chassis->Motors3508.motor[2], 0 , Motor3508, canx, 0x203);
	MotorInit(&chassis->Motors3508.motor[3], 0 , Motor3508, canx, 0x204);
	
//7583	4626   5912
//	MotorInit(&chassis->Motors6020.motor[0], 6821 , Motor6020, canx, 0x205);
//	MotorInit(&chassis->Motors6020.motor[1], 7520 , Motor6020, canx, 0x206);
//	MotorInit(&chassis->Motors6020.motor[2], 5460,   Motor6020, canx, 0x207);
//	MotorInit(&chassis->Motors6020.motor[3], 4731 , Motor6020, canx, 0x208);
	MotorInit(&chassis->Motors6020.motor[0], 3902 , Motor6020, canx, 0x205);
	MotorInit(&chassis->Motors6020.motor[1], 5167 , Motor6020, canx, 0x206);
	MotorInit(&chassis->Motors6020.motor[2], 5140 , Motor6020, canx, 0x207);
	MotorInit(&chassis->Motors6020.motor[3], 2389 , Motor6020, canx, 0x208);
	
	BasePID_Init(&chassis->Motors3508.RunPID[0], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[1], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[2], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[3], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	
	BasePID_Init(&chassis->Motors6020.TurnPID[0], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors6020.TurnPID[1], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors6020.TurnPID[2], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors6020.TurnPID[3], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	
	chassis->Movement.Sensitivity.Vx 	   = 5;
	chassis->Movement.Sensitivity.Vy     = 5;
	chassis->Movement.Sensitivity.Omega  = 4;
	chassis->Movement.ModuleOfSpeed  	   = 0;  
	chassis->Movement.AngleOfSpeed 	 	   = 0;		
	
}




/**
  * @brief  ���ݶ��������ǶȺͷ����Ƕȵļн��ж���СĿ��Ƕ�
  */
float FindBestTargetAngle(float targetAngle, float feedbackAngle, uint8_t* flag)
{
	float vectorDeltaAngle;
	float deltaAngle;
	vectorDeltaAngle = targetAngle - feedbackAngle;
	
	if((vectorDeltaAngle)>180)
		vectorDeltaAngle = vectorDeltaAngle - 360.0f;
	else if ((vectorDeltaAngle)<-180)
		vectorDeltaAngle = 360 + vectorDeltaAngle;
		
	if(targetAngle >= 0.0f)
	{
		deltaAngle = vectorDeltaAngle - 180.0f;
		if((deltaAngle)>180)
			deltaAngle = deltaAngle - 360.0f;
		else if ((deltaAngle)<-180)
			deltaAngle = 360 + deltaAngle;
	}
	else if(targetAngle < 0.0f)
	{
		deltaAngle = vectorDeltaAngle + 180.0f;
		if((deltaAngle)>180)
			deltaAngle = deltaAngle - 360.0f;
		else if ((deltaAngle)<-180)
			deltaAngle = 360 + deltaAngle;
	}
	
	if(myABS(vectorDeltaAngle) > myABS(deltaAngle))
	{
		if(targetAngle >= 0.0f)
			targetAngle = targetAngle - 180.0f;
		else if(targetAngle < 0.0f)
			targetAngle = targetAngle + 180.0f;
		*flag = 1;
	}
	else 
	{
		*flag = 0;
	}
	return targetAngle;
}


/**
  * @brief  ��ȡ���ջ��������ݣ����ݶ����˶��߼�����
  */
void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Chassis_Attitude_Info* Swerve, float canAngle)
{
	{
		float angle = AtR * (-canAngle);
//(1024-660)*5=1832
//		chassis->Movement.Vx     			 = (rc_ctrl->rc.ch1 - 1024) * chassis->Movement.Sensitivity.Vx;	
//		chassis->Movement.Vy		 		   = -(rc_ctrl->rc.ch0 - 1024) * chassis->Movement.Sensitivity.Vy;	
//		if(rc_ctrl->rc.sw >1400){
//		chassis->Movement.Omega 			 =rc_ctrl->rc.sw * 3; //-(rc_ctrl->rc.ch2 - 1024) * chassis->Movement.Sensitivity.Omega;
//		}
//		else chassis->Movement.Omega 			 = 0 ;
		int16_t rawVx = chassis->Movement.Vx;
		int16_t rawVy = chassis->Movement.Vy;
		chassis->Movement.Vx = (rawVx * cos(angle) - rawVy * sin(angle));
		chassis->Movement.Vy = (rawVx * sin(angle) + rawVy * cos(angle));
		
		VectorSolve(chassis->Movement.Vx, chassis->Movement.Vy, &chassis->Movement.AngleOfSpeed, &chassis->Movement.ModuleOfSpeed,8);
		
		//< ������ջ���������ת������ƽ���ٶȺ��������ĸ��������Ϸֽ�ĽǶ�
		chassis->Vectors.Vx[0] =  chassis->Movement.Vx + chassis->Movement.Omega;
		chassis->Vectors.Vy[0] =  chassis->Movement.Vy + chassis->Movement.Omega;
		//-------------------------------------------
		chassis->Vectors.Vx[1] =  chassis->Movement.Vx + chassis->Movement.Omega;
		chassis->Vectors.Vy[1] =  chassis->Movement.Vy - chassis->Movement.Omega;
		chassis->Vectors.Vx[2] =  chassis->Movement.Vx - chassis->Movement.Omega;
		chassis->Vectors.Vy[2] =  chassis->Movement.Vy - chassis->Movement.Omega;
		chassis->Vectors.Vx[3] =  chassis->Movement.Vx - chassis->Movement.Omega;
		chassis->Vectors.Vy[3] =  chassis->Movement.Vy + chassis->Movement.Omega;		
		
		//< ���ݽ��ջ�ˮƽ�ʹ�ֱ����ת���ٶȷ�������������ģ�ͼн� 
		VectorSolve(chassis->Vectors.Vx[0], chassis->Vectors.Vy[0], &chassis->Vectors.Angle[0] , &chassis->Vectors.SpeedNo[0],0);
		VectorSolve(chassis->Vectors.Vx[1], chassis->Vectors.Vy[1], &chassis->Vectors.Angle[1] , &chassis->Vectors.SpeedNo[1],1);
		VectorSolve(chassis->Vectors.Vx[2], chassis->Vectors.Vy[2], &chassis->Vectors.Angle[2] , &chassis->Vectors.SpeedNo[2],2);
		VectorSolve(chassis->Vectors.Vx[3], chassis->Vectors.Vy[3], &chassis->Vectors.Angle[3] , &chassis->Vectors.SpeedNo[3],3);
		
		
		//< ��ң���������Ŀ��Ƕ��뵱ǰ�����ĵ���ǶȶԱȣ�ѡ����뷴���Ƕ������Ŀ��Ƕȣ������жϵ������ת�Ƿ���Ҫ�ı�
		chassis->Vectors.BestAngle[0] = FindBestTargetAngle(chassis->Vectors.Angle[0], chassis->Motors6020.motor[0].Data.Angle, &chassis->Vectors.SpeedChangeFlag[0]);
		chassis->Vectors.BestAngle[1] = FindBestTargetAngle(chassis->Vectors.Angle[1], chassis->Motors6020.motor[1].Data.Angle, &chassis->Vectors.SpeedChangeFlag[1]);
		chassis->Vectors.BestAngle[2] = FindBestTargetAngle(chassis->Vectors.Angle[2], chassis->Motors6020.motor[2].Data.Angle, &chassis->Vectors.SpeedChangeFlag[2]);
		chassis->Vectors.BestAngle[3] = FindBestTargetAngle(chassis->Vectors.Angle[3], chassis->Motors6020.motor[3].Data.Angle, &chassis->Vectors.SpeedChangeFlag[3]);
		
		chassis->Vectors.TargetEcd[0] = chassis->Motors6020.motor[0].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[0]/(float)360.0f);
		chassis->Vectors.TargetEcd[1] = chassis->Motors6020.motor[1].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[1]/(float)360.0f);
		chassis->Vectors.TargetEcd[2] = chassis->Motors6020.motor[2].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[2]/(float)360.0f);
		chassis->Vectors.TargetEcd[3] = chassis->Motors6020.motor[3].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[3]/(float)360.0f);
		
				 
		//< ��Ŀ��Ƕȵı�����ֵΪ��㣬��������ǰ�����Ƕȱ�����ֵ��Ӧ�ĽǶ� (-180 <- 0 -> 180)
		chassis->Vectors.FeedbackAngle[0] = EcdtoAngle(chassis->Vectors.TargetEcd[0], chassis->Motors6020.motor[0].Data.RawEcd);
		chassis->Vectors.FeedbackAngle[1] = EcdtoAngle(chassis->Vectors.TargetEcd[1], chassis->Motors6020.motor[1].Data.RawEcd);
		chassis->Vectors.FeedbackAngle[2] = EcdtoAngle(chassis->Vectors.TargetEcd[2], chassis->Motors6020.motor[2].Data.RawEcd);
		chassis->Vectors.FeedbackAngle[3] = EcdtoAngle(chassis->Vectors.TargetEcd[3], chassis->Motors6020.motor[3].Data.RawEcd);
	}
}

/**
  * @brief ����PID���Ƴ�ʼ��
  */
void SwervePowerConrolInit(SwerveChassis* chassis, BasePID_Object base_pid, BasePID_Object power_pid)
	{
    BasePID_Init(&chassis->Power.PowerPID[0], base_pid.Kp, base_pid.Ki, base_pid.Kd, base_pid.KiPartDetachment);
	  BasePID_Init(&chassis->Power.PowerPID[1], power_pid.Kp, power_pid.Ki, power_pid.Kd, power_pid.KiPartDetachment);
  }

/**
  * @brief  �ڶ������˶�ѧ����󱻵��ã�����3508��6020���������ֵ��������CAN����֡��0x200��0x1ff��������֡
  */
void SwerveChassisMotionControl(SwerveChassis* chassis ,RC_Ctrl* rc_ctrl ,Holder_t* holder)
{
	for(int j =0; j<4; j++)
	{
		float deltaAngle = chassis->Vectors.BestAngle[j] -chassis->Vectors.Angle[j];

		if(chassis->Vectors.SpeedChangeFlag[j] == 1)
			chassis->Vectors.SpeedNo[j] = - chassis->Vectors.SpeedNo[j];
		else
			chassis->Vectors.SpeedNo[j] = chassis->Vectors.SpeedNo[j];
		
		//< ���ID��װ˳�����
//		if(j == 2)
//			chassis->Motors3508.motor[j+1].Data.Target = chassis->Vectors.SpeedNo[j];
//		else if(j == 3)
//			chassis->Motors3508.motor[j-1].Data.Target = chassis->Vectors.SpeedNo[j];
//		else
		if(j == 1)
			chassis->Motors3508.motor[j].Data.Target = chassis->Vectors.SpeedNo[j];
		else if(j == 2)
			chassis->Motors3508.motor[j].Data.Target = chassis->Vectors.SpeedNo[j];
		else
			chassis->Motors3508.motor[j].Data.Target = -chassis->Vectors.SpeedNo[j];
	}
	chassis->Power.Max_Power = BasePID_BaseControl((BasePID_Object*)(chassis->Power.PowerPID+0),     referee2022.power_heat_data.chassis_power_buffer,   referee2022.power_heat_data.chassis_power);
	for(int i=0;i<4;i++)  //< ������̵��
	{   
		if( chassis->Motors3508.motor[i].Data.Target >  7800)   chassis->Motors3508.motor[i].Data.Target =  7800;
		if( chassis->Motors3508.motor[i].Data.Target < -7800)   chassis->Motors3508.motor[i].Data.Target = -7800;
		chassis->Motors3508.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors3508.RunPID + i), chassis->Motors3508.motor[i].Data.Target, chassis->Motors3508.motor[i].Data.SpeedRPM);
	}
	chassis->Power.Target_Power_Sum[0] =myABS(chassis->Motors3508.motor[0].Data.Output)+myABS(chassis->Motors3508.motor[1].Data.Output)+myABS(chassis->Motors3508.motor[2].Data.Output)+myABS(chassis->Motors3508.motor[3].Data.Output)+myABS(chassis->Motors6020.motor[0].Data.Output)+myABS(chassis->Motors6020.motor[1].Data.Output)+myABS(chassis->Motors6020.motor[2].Data.Output)+myABS(chassis->Motors6020.motor[3].Data.Output);
	chassis ->Power.bili = chassis->Power.Target_Power_Sum[0]/(chassis->Power.Max_Power);
     if(fly_flag==0)
		 {
	        chassis->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 0), -chassis->Vectors.FeedbackAngle[0], 0, 0.1047f*chassis->Motors6020.motor[0].Data.SpeedRPM); //< RPMת��Ϊrad/s ��λת��ϵ��
	        chassis->Motors6020.motor[1].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 1), -chassis->Vectors.FeedbackAngle[1], 0, 0.1047f*chassis->Motors6020.motor[1].Data.SpeedRPM);
	        chassis->Motors6020.motor[2].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 2), -chassis->Vectors.FeedbackAngle[2], 0, 0.1047f*chassis->Motors6020.motor[2].Data.SpeedRPM);
	        chassis->Motors6020.motor[3].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 3), -chassis->Vectors.FeedbackAngle[3], 0, 0.1047f*chassis->Motors6020.motor[3].Data.SpeedRPM);
		 }
		 else if((fly_flag==1)&&(Holder.Direction_Flag==0))
		 {
			    chassis->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 0), fly_angle[0]*AtR, chassis->Motors6020.motor[0].Data.Angle, 0.1047f*chassis->Motors6020.motor[0].Data.SpeedRPM); //< RPMת��Ϊrad/s ��λת��ϵ��
	        chassis->Motors6020.motor[1].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 1), fly_angle[1]*AtR, chassis->Motors6020.motor[1].Data.Angle, 0.1047f*chassis->Motors6020.motor[1].Data.SpeedRPM);
	        chassis->Motors6020.motor[2].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 2), fly_angle[2]*AtR, chassis->Motors6020.motor[2].Data.Angle, 0.1047f*chassis->Motors6020.motor[2].Data.SpeedRPM);
	        chassis->Motors6020.motor[3].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 3), fly_angle[3]*AtR, chassis->Motors6020.motor[3].Data.Angle, 0.1047f*chassis->Motors6020.motor[3].Data.SpeedRPM);
		 }
		  else if((fly_flag==1)&&(Holder.Direction_Flag==1))
		 {
			    chassis->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 0), fly_angle2[0]*AtR, chassis->Motors6020.motor[0].Data.Angle, 0.1047f*chassis->Motors6020.motor[0].Data.SpeedRPM); //< RPMת��Ϊrad/s ��λת��ϵ��
	        chassis->Motors6020.motor[1].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 1), fly_angle2[1]*AtR, chassis->Motors6020.motor[1].Data.Angle, 0.1047f*chassis->Motors6020.motor[1].Data.SpeedRPM);
	        chassis->Motors6020.motor[2].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 2), fly_angle2[2]*AtR, chassis->Motors6020.motor[2].Data.Angle, 0.1047f*chassis->Motors6020.motor[2].Data.SpeedRPM);
	        chassis->Motors6020.motor[3].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 3), fly_angle2[3]*AtR, chassis->Motors6020.motor[3].Data.Angle, 0.1047f*chassis->Motors6020.motor[3].Data.SpeedRPM);
		 }
			if(chassis->Power.Target_Power_Sum[0]>chassis->Power.Max_Power)
			{
					chassis->Power.speedfinalout[0]= (chassis->Motors3508.motor[0].Data.Output)/chassis->Power.bili;
				  chassis->Power.speedfinalout[1]= (chassis->Motors3508.motor[1].Data.Output)/chassis->Power.bili;
				  chassis->Power.speedfinalout[2]= (chassis->Motors3508.motor[2].Data.Output)/chassis->Power.bili;
				  chassis->Power.speedfinalout[3]= (chassis->Motors3508.motor[3].Data.Output)/chassis->Power.bili;
				  chassis->Power.anglefinalout[0]= chassis->Motors6020.motor[0].Data.Output/chassis->Power.bili;
			    chassis->Power.anglefinalout[1]= chassis->Motors6020.motor[1].Data.Output/chassis->Power.bili;
		      chassis->Power.anglefinalout[2]= chassis->Motors6020.motor[2].Data.Output/chassis->Power.bili;
		      chassis->Power.anglefinalout[3]= chassis->Motors6020.motor[3].Data.Output/chassis->Power.bili;
			}
			else
			{	
					chassis->Power.speedfinalout[0]= chassis->Motors3508.motor[0].Data.Output;
				  chassis->Power.speedfinalout[1]= chassis->Motors3508.motor[1].Data.Output;
				  chassis->Power.speedfinalout[2]= chassis->Motors3508.motor[2].Data.Output;
				  chassis->Power.speedfinalout[3]= chassis->Motors3508.motor[3].Data.Output;
					chassis->Power.anglefinalout[0]= chassis->Motors6020.motor[0].Data.Output;
			    chassis->Power.anglefinalout[1]= chassis->Motors6020.motor[1].Data.Output;
			    chassis->Power.anglefinalout[2]= chassis->Motors6020.motor[2].Data.Output;
			    chassis->Power.anglefinalout[3]= chassis->Motors6020.motor[3].Data.Output;
			}
    if(chassis->SuperPowerMode == 0)
    {
		for(int i=0;i<4;i++) {
//			if(rc_ctrl->rc.sw<1400&&rc_ctrl->rc.sw>600&&(rc_ctrl->rc.ch2-1024)*holder->Yaw.Sensitivity>15&&(rc_ctrl->rc.ch2-1024)*holder->Yaw.Sensitivity<-15){
//			if(chassis->Power.speedfinalout[i]>5000){
//				chassis->Power.speedfinalout[i] = 5000 ;
//      }				
//			if(chassis->Power.speedfinalout[i]<-5000 ){
//				chassis->Power.speedfinalout[i] = -5000 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==50&&chassis->Power.speedfinalout[i] > 1600){
//			chassis->Power.speedfinalout[i] = 1600 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==50&&chassis->Power.speedfinalout[i] < -1600){
//			chassis->Power.speedfinalout[i] = -1600 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==60&&chassis->Power.speedfinalout[i] >1700){
//			chassis->Power.speedfinalout[i] = 1700 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==60&&chassis->Power.speedfinalout[i] < 1700){
//			chassis->Power.speedfinalout[i] = -1700 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==70&&chassis->Power.speedfinalout[i] > 1800){
//			chassis->Power.speedfinalout[i] = 1800 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==70&&chassis->Power.speedfinalout[i] < -1800){
//			chassis->Power.speedfinalout[i] = -1800 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==80&&chassis->Power.speedfinalout[i] > 2000){
//			chassis->Power.speedfinalout[i] = 2000 ; 
//			}
//			if(rc_ctrl->rc.sw>1400&&referee2022.game_robot_status.chassis_power_limit ==80&&chassis->Power.speedfinalout[i] < -2000){
//			chassis->Power.speedfinalout[i] = -2000 ; 
//			}

      MotorFillData(&chassis->Motors3508.motor[i], chassis->Power.speedfinalout[i]);
 //		MotorFillData(&chassis->Motors3508.motor[i], chassis->Motors3508.motor[i].Data.Output);
		}
	if(referee2022.power_heat_data.chassis_power_buffer<20)
		{
			MotorFillData(&chassis->Motors6020.motor[0],chassis->Power.anglefinalout[0]);
	    MotorFillData(&chassis->Motors6020.motor[1],chassis->Power.anglefinalout[1]);	
	    MotorFillData(&chassis->Motors6020.motor[2],chassis->Power.anglefinalout[2]);
	    MotorFillData(&chassis->Motors6020.motor[3],chassis->Power.anglefinalout[3]);		
		}
		else
		{
		  MotorFillData(&chassis->Motors6020.motor[0],chassis->Motors6020.motor[0].Data.Output);
	    MotorFillData(&chassis->Motors6020.motor[1],chassis->Motors6020.motor[1].Data.Output);	
    	MotorFillData(&chassis->Motors6020.motor[2],chassis->Motors6020.motor[2].Data.Output);
    	MotorFillData(&chassis->Motors6020.motor[3],chassis->Motors6020.motor[3].Data.Output);
		}
		}
		else if(chassis->SuperPowerMode == 1)
		{
		for(int i=0;i<4;i++) {
			MotorFillData(&chassis->Motors3508.motor[i], chassis->Motors3508.motor[i].Data.Output);
		}
    	MotorFillData(&chassis->Motors6020.motor[0],chassis->Motors6020.motor[0].Data.Output);
	    MotorFillData(&chassis->Motors6020.motor[1],chassis->Motors6020.motor[1].Data.Output);	
    	MotorFillData(&chassis->Motors6020.motor[2],chassis->Motors6020.motor[2].Data.Output);
    	MotorFillData(&chassis->Motors6020.motor[3],chassis->Motors6020.motor[3].Data.Output);
		}
//	        chassis->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 0), -chassis->Vectors.FeedbackAngle[0], 0, 0.1047f*chassis->Motors6020.motor[0].Data.SpeedRPM); //< RPMת��Ϊrad/s ��λת��ϵ��
//	        chassis->Motors6020.motor[1].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 1), -chassis->Vectors.FeedbackAngle[1], 0, 0.1047f*chassis->Motors6020.motor[1].Data.SpeedRPM);
//	        chassis->Motors6020.motor[2].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 2), -chassis->Vectors.FeedbackAngle[2], 0, 0.1047f*chassis->Motors6020.motor[2].Data.SpeedRPM);
//	        chassis->Motors6020.motor[3].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 3), -chassis->Vectors.FeedbackAngle[3], 0, 0.1047f*chassis->Motors6020.motor[3].Data.SpeedRPM);

//	        chassis->Power.Target_Power_Sum[1] =myABS(chassis->Motors6020.motor[0].Data.Output)+myABS(chassis->Motors6020.motor[1].Data.Output)+myABS(chassis->Motors6020.motor[2].Data.Output)+myABS(chassis->Motors6020.motor[3].Data.Output);
//          chassis->Power.bilibili = chassis->Power.Target_Power_Sum[1]/(0.6*chassis->Power.Max_Power);
//	if(chassis->Power.Target_Power_Sum[1]>0.6*chassis->Power.Max_Power){
//					chassis->Power.anglefinalout[0]= chassis->Motors6020.motor[0].Data.Output/chassis->Power.bilibili;
//			    chassis->Power.anglefinalout[1]= chassis->Motors6020.motor[1].Data.Output/chassis->Power.bilibili;
//		      chassis->Power.anglefinalout[2]= chassis->Motors6020.motor[2].Data.Output/chassis->Power.bilibili;
//		      chassis->Power.anglefinalout[3]= chassis->Motors6020.motor[3].Data.Output/chassis->Power.bilibili;
//	}
//			else
//			{	

//					chassis->Power.anglefinalout[0]= chassis->Motors6020.motor[0].Data.Output;
//			    chassis->Power.anglefinalout[1]= chassis->Motors6020.motor[1].Data.Output;
//			    chassis->Power.anglefinalout[2]= chassis->Motors6020.motor[2].Data.Output;
//			    chassis->Power.anglefinalout[3]= chassis->Motors6020.motor[3].Data.Output;

//			}
//	MotorFillData(&chassis->Motors6020.motor[0],chassis->Motors6020.motor[0].Data.Output);
//	MotorFillData(&chassis->Motors6020.motor[1],chassis->Motors6020.motor[1].Data.Output);	
//	MotorFillData(&chassis->Motors6020.motor[2],chassis->Motors6020.motor[2].Data.Output);
//	MotorFillData(&chassis->Motors6020.motor[3],chassis->Motors6020.motor[3].Data.Output);
//	MotorFillData(&chassis->Motors6020.motor[0],chassis->Power.anglefinalout[0]);
//	MotorFillData(&chassis->Motors6020.motor[1],chassis->Power.anglefinalout[1]);	
//	MotorFillData(&chassis->Motors6020.motor[2],chassis->Power.anglefinalout[2]);
//	MotorFillData(&chassis->Motors6020.motor[3],chassis->Power.anglefinalout[3]);
	
		}

/**
* @brief ���ݽ��ջ��Ƿ����ߣ��ж��Ƿ����������̿����źš����̵�CAN����ȫ������0x1ff��0x200����CAN����֡
  */
void SwerveChassisOutputControl(SwerveChassis* chassis, RC_Ctrl rcCtrl)
{
	if(rcCtrl.isOnline == 1) 
	{
		MotorCanOutput(can2, 0x1ff);  
		MotorCanOutput(can2, 0x200);		
	}
	else 
	{ //< ���ջ����ߣ�������Ϊ0
		MotorFillData(&chassis->Motors6020.motor[0],0);
		MotorFillData(&chassis->Motors6020.motor[1],0);
		MotorFillData(&chassis->Motors6020.motor[2],0);
		MotorFillData(&chassis->Motors6020.motor[3],0);
		MotorFillData(&chassis->Motors3508.motor[0],0);
		MotorFillData(&chassis->Motors3508.motor[1],0);
		MotorFillData(&chassis->Motors3508.motor[2],0);
		MotorFillData(&chassis->Motors3508.motor[3],0);
		MotorCanOutput(can2, 0x1ff);
		MotorCanOutput(can2, 0x200);
	}
}


/**
  * @brief  �޸Ķ��ֵ���PID�����Ľӿں���
  */
//void ChangeChassisPID(SwerveChassis* chassis, PIDParameters pid)
//{
//	for(int i =0;i<4;i++)
//	{
//		chassis->Motors6020.TurnPID[i].Kp = pid.TurnKp;
//		chassis->Motors6020.TurnPID[i].Ki = pid.TurnKi;
//		chassis->Motors6020.TurnPID[i].Kd = pid.TurnKd;
//		chassis->Motors3508.RunPID[i].Kp  = pid.SpeedKp;
//		chassis->Motors3508.RunPID[i].Ki  = pid.SpeedKi;
//		chassis->Motors3508.RunPID[i].Kd  = pid.SpeedKd;
//	}	
//}

/**
  * @brief ���̸���pid����
  */
void SwerveChassisSetFollowPID(SwerveChassis* chassis, BasePID_Object follow_pid)
{
		BasePID_Init(&chassis->Motors3508.FollowPID[0], follow_pid.Kp, follow_pid.Ki, follow_pid.Kd, follow_pid.KiPartDetachment);
}

/**
  * @brief �����˶�ģʽ����
  */
void Chassis_Mode_Control(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Holder_t* holder , Chassis_Attitude_Info* Swerve,Supercap* Cap)
{    
	  Output_6020 = 0;
	  for(int i=0;i<4;i++)
	     Output_6020 += myABS(chassis->Motors6020.motor[i].Data.Output);
	  if(Output_6020<=4000)
		{
			rush_cut++;
			if(rush_cut>=5)
			{
				rush_flag=1;
			  rush_cut =5;
			}
			else
				rush_flag=0;
		}
		else
		{
			rush_cut=0;
			rush_flag=0;
		}
  	if(Swerve->Roll_Flag==1||Swerve->Roll_Flag1==1) //���̱����� С����ģʽ
		{
		  if(Holder.Direction_Flag==0)
			{
		  chassis->Movement.Vx=Speed_Limit*(rc_ctrl->Chassis_Y_Integ);//����ģʽʹ��б�º���   //ǰ��
		  chassis->Movement.Vy=-Speed_Limit*(rc_ctrl->Chassis_X_Integ);	
			}
			else if(Holder.Direction_Flag==1)
			{
		  chassis->Movement.Vx=-Speed_Limit*(rc_ctrl->Chassis_X_Integ);
		  chassis->Movement.Vy=-Speed_Limit*(rc_ctrl->Chassis_Y_Integ);	
			}
			
      if(referee2022.game_robot_status.chassis_power_limit == 50)
			{
			  chassis->Movement.Omega =2700;
				if(chassis->Movement.Vx>1213){
		    chassis->Movement.Vx=1213;
		    }
		    if(chassis->Movement.Vx<-1213){
		    chassis->Movement.Vx=-1213;
		    }				
		    if(chassis->Movement.Vy>1213){
		    chassis->Movement.Vy=1213;
		    }
		    if(chassis->Movement.Vy<-1213){
		    chassis->Movement.Vy=-1213;
		    }	
			}
			if(referee2022.game_robot_status.chassis_power_limit == 60)
			{
			  chassis->Movement.Omega =  3000;
				if(chassis->Movement.Vx>1952){
		    chassis->Movement.Vx=1952;
		    }
		    if(chassis->Movement.Vx<-1952){
		    chassis->Movement.Vx=-1952;
		    }				
		    if(chassis->Movement.Vy>1952){
		    chassis->Movement.Vy=1952;
		    }
		    if(chassis->Movement.Vy<-1952){
		    chassis->Movement.Vy=-1952;
			  }
		  }
			if(referee2022.game_robot_status.chassis_power_limit == 80)
			{
			  chassis->Movement.Omega =  3600;
				if(chassis->Movement.Vx>3512){
		    chassis->Movement.Vx=3512;
		    }
		    if(chassis->Movement.Vx<-3512){
		    chassis->Movement.Vx=-3512;
		    }				
		    if(chassis->Movement.Vy>3512){
		    chassis->Movement.Vy=3512;
		    }
		    if(chassis->Movement.Vy<-3512){
		    chassis->Movement.Vy=-3512;
			  }
			}
			if(referee2022.game_robot_status.chassis_power_limit == 100)  
			{
			  chassis->Movement.Omega =  3600;
				if(chassis->Movement.Vx>4152){
		    chassis->Movement.Vx=4152;
		    }
		    if(chassis->Movement.Vx<-4152){
		    chassis->Movement.Vx=-4152;
		    }				
		    if(chassis->Movement.Vy>4152){
		    chassis->Movement.Vy=4152;
		    }
		    if(chassis->Movement.Vy<-4152){
		    chassis->Movement.Vy=-4152;
			  }
			}

			SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve,-holder->Motors6020.motor[0].Data.Angle);		
/*
//			if(referee2022.game_robot_status.chassis_power_limit == 50)
//			{
//			  chassis->Movement.Omega =  2700;
//       for(int i=0 ; i < 4 ; i++)
//				{
//				if(chassis->Vectors.SpeedNo[i]>4750)
//          {
//					   chassis->Vectors.SpeedNo[i]=4750;
//					}
//				if(chassis->Vectors.SpeedNo[i]<-4750)
//          {
//					   chassis->Vectors.SpeedNo[i]=-4750;
//					}	
//				}
//			}
//			if(referee2022.game_robot_status.chassis_power_limit == 60)
//			{
//				chassis->Movement.Omega =  3000;
//       for(int i=0 ; i < 4 ; i++)
//				{
//				if(chassis->Vectors.SpeedNo[i]>5295)
//          {
//					   chassis->Vectors.SpeedNo[i]=5295;
//					}
//				if(chassis->Vectors.SpeedNo[i]<-5295)
//          {
//					   chassis->Vectors.SpeedNo[i]=-5295;
//					}	
//				}
//			}
//		 if(referee2022.game_robot_status.chassis_power_limit == 80)
//			{
//				chassis->Movement.Omega =  3600;
//       for(int i=0 ; i < 4 ; i++)
//				{
//				if(chassis->Vectors.SpeedNo[i]>6516)
//          {
//					   chassis->Vectors.SpeedNo[i]=6516;
//					}
//				if(chassis->Vectors.SpeedNo[i]<-6516)
//          {
//					   chassis->Vectors.SpeedNo[i]=-6516;
//					}	
//				}
//			}
//			if(referee2022.game_robot_status.chassis_power_limit == 100)
//			{
//				chassis->Movement.Omega =  3600;
//       for(int i=0 ; i < 4 ; i++)
//				{
//				if(chassis->Vectors.SpeedNo[i]>7874)
//          {
//					   chassis->Vectors.SpeedNo[i]=7874;
//					}
//				if(chassis->Vectors.SpeedNo[i]<-7874)
//          {
//					   chassis->Vectors.SpeedNo[i]=-7874;
//					}	
//				}
//			}
*/
		  SwerveChassisMotionControl(&swerveChassis ,&rc_Ctrl , &Holder); 
		}
/*
//		else if(rc_ctrl->rc.sw<600) //Ť��ģʽ
//		{
//			chassis->Movement.Omega =0; //BasePID_AngleControl((BasePID_Object*)(chassis->Motors3508.FollowPID+0),0,holder->Motors6020.motor[0].Data.Angle, holder->Motors6020.motor[0].Data.SpeedRPM);
//			SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve, -holder->Motors6020.motor[0].Data.Angle);		
//		  SwerveChassisMotionControl(&swerveChassis,&rc_Ctrl , &Holder); 
//		}	
*/		
		else if(Swerve->Roll_Flag==0||Swerve->Roll_Flag1==0)
		{
			if(Holder.Direction_Flag==0)
			{
		  chassis->Movement.Vx=Speed_Limit*(rc_ctrl->Chassis_Y_Integ);//����ģʽʹ��б�º���   //ǰ��
		  chassis->Movement.Vy=-Speed_Limit*(rc_ctrl->Chassis_X_Integ);	
			}
			else if(Holder.Direction_Flag==1)
			{
		  chassis->Movement.Vx=-Speed_Limit*(rc_ctrl->Chassis_X_Integ);
		  chassis->Movement.Vy=-Speed_Limit*(rc_ctrl->Chassis_Y_Integ);	
			}
			
/*
//			if(Cap->cap_state.Supercap_Flag == 1)
//			{
//		  if(chassis->Movement.Vx>7832){
//		  chassis->Movement.Vx=7832;
//		  }
//		  if(chassis->Movement.Vx<-7832){
//		  chassis->Movement.Vx=-7832;
//		  }				
//		  if(chassis->Movement.Vy>7832){
//		  chassis->Movement.Vy=7832;
//		  }
//		  if(chassis->Movement.Vy<-7832){
//		  chassis->Movement.Vy=-7832;
//		  }
//		  }
//			if(Cap->cap_state.Supercap_Flag == 0)
//      {
//	  	if(rush_flag==0)  //��6020����ϴ�ʱѹ��3508���������ȱ�֤������ת��˿��
//			{
//			if(referee2022.game_robot_status.chassis_power_limit == 50){
//			if(chassis->Movement.Vx>3632){
//		  chassis->Movement.Vx=3632;
//		  }
//		  if(chassis->Movement.Vx<-3632){
//		  chassis->Movement.Vx=-3632;
//		  }				
//		  if(chassis->Movement.Vy>3632){
//		  chassis->Movement.Vy=3632;
//		  }
//		  if(chassis->Movement.Vy<-3632){
//		  chassis->Movement.Vy=-3632;
//		  }
//		  }
//			if(referee2022.game_robot_status.chassis_power_limit == 60){
//			if(chassis->Movement.Vx>3932){
//		  chassis->Movement.Vx=3932;
//		  }
//		  if(chassis->Movement.Vx<-3932){
//		  chassis->Movement.Vx=-3932;
//		  }				
//		  if(chassis->Movement.Vy>3932){
//		  chassis->Movement.Vy=3932;
//		  }
//		  if(chassis->Movement.Vy<-3932){
//		  chassis->Movement.Vy=-3932;
//		  }
//			}
//		  if(referee2022.game_robot_status.chassis_power_limit == 80){  
//			if(chassis->Movement.Vx>4472){
//		  chassis->Movement.Vx=4472;
//		  }
//		  if(chassis->Movement.Vx<-4472){
//		  chassis->Movement.Vx=-4472;
//		  }				
//		  if(chassis->Movement.Vy>4472){
//		  chassis->Movement.Vy=4472;
//		  }
//		  if(chassis->Movement.Vy<-4472){
//		  chassis->Movement.Vy=-4472;
//		  }
//			}
//			if(referee2022.game_robot_status.chassis_power_limit == 100){
//			if(chassis->Movement.Vx>5332){
//		  chassis->Movement.Vx=5332;
//		  }
//		  if(chassis->Movement.Vx<-5332){
//		  chassis->Movement.Vx=-5332;
//		  }				
//		  if(chassis->Movement.Vy>5332){
//		  chassis->Movement.Vy=5332;
//		  }
//		  if(chassis->Movement.Vy<-5332){
//		  chassis->Movement.Vy=-5332;
//		  }
//			}
//		  }
//			else if(rush_flag==1)   //��6020��������ʱ���3508����ĵ������ѻ�����������
//		 {
//			 if(referee2022.game_robot_status.chassis_power_limit == 50)
//			{
//				if(chassis->Movement.Vx>4982){
//		    chassis->Movement.Vx=4982;
//		    }
//		    if(chassis->Movement.Vx<-4982){
//		    chassis->Movement.Vx=-4982;
//		    }				
//		    if(chassis->Movement.Vy>4982){
//		    chassis->Movement.Vy=4982;
//		    }
//		    if(chassis->Movement.Vy<-4982){
//		    chassis->Movement.Vy=-4982;
//		    }	
//			}
//			if(referee2022.game_robot_status.chassis_power_limit == 60)
//			{
//				if(chassis->Movement.Vx>7452){
//		    chassis->Movement.Vx=7452;
//		    }
//		    if(chassis->Movement.Vx<-7452){
//		    chassis->Movement.Vx=-7452;
//		    }				
//		    if(chassis->Movement.Vy>7452){
//		    chassis->Movement.Vy=7452;
//		    }
//		    if(chassis->Movement.Vy<-7452){
//		    chassis->Movement.Vy=-7452;
//			  }
//		  }
//			if(referee2022.game_robot_status.chassis_power_limit == 80)
//			{
//				if(chassis->Movement.Vx>5912){
//		    chassis->Movement.Vx=5912;
//		    }
//		    if(chassis->Movement.Vx<-5912){
//		    chassis->Movement.Vx=-5912;
//		    }				
//		    if(chassis->Movement.Vy>5912){
//		    chassis->Movement.Vy=5912;
//		    }
//		    if(chassis->Movement.Vy<-5912){
//		    chassis->Movement.Vy=-5912;
//			  }
//			}
//			if(referee2022.game_robot_status.chassis_power_limit == 100)
//			{
//				if(chassis->Movement.Vx>6412){
//		    chassis->Movement.Vx=6412;
//		    }
//		    if(chassis->Movement.Vx<-6412){
//		    chassis->Movement.Vx=-6412;
//		    }				
//		    if(chassis->Movement.Vy>6412){
//		    chassis->Movement.Vy=6412;
//		    }
//		    if(chassis->Movement.Vy<-6412){
//		    chassis->Movement.Vy=-6412;
//			  }
//			}
//		  }
*/
			chassis->Movement.Omega =BasePID_AngleControlFollow((BasePID_Object*)(chassis->Motors3508.FollowPID+0),0,holder->Motors6020.motor[0].Data.Angle, holder->Motors6020.motor[0].Data.SpeedRPM);
			SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve, -holder->Motors6020.motor[0].Data.Angle);	
//			if(Cap->cap_state.Supercap_Flag == 1)
//			{
//			 if(rush_flag==0)
//       for(int i=0 ; i < 4 ; i++)
//				{
//				if(chassis->Vectors.SpeedNo[i]>12000)
//          {
//					   chassis->Vectors.SpeedNo[i]=12000;
//					}
//				if(chassis->Vectors.SpeedNo[i]<-12000)
//          {
//						
//					   chassis->Vectors.SpeedNo[i]=-12000;
//					}	
//				}
//			}
      if(Cap->cap_state.Supercap_Flag == 0)
      {
/*
			if(referee2022.game_robot_status.chassis_power_limit == 50)
			{
			 if(rush_flag==0)
       for(int i=0 ; i < 4 ; i++)
				{
				if(chassis->Vectors.SpeedNo[i]>3729)
          {
					   chassis->Vectors.SpeedNo[i]=3729;
					}
				if(chassis->Vectors.SpeedNo[i]<-3729)
          {
					   chassis->Vectors.SpeedNo[i]=-3729;
					}	
				}
			}
			if(referee2022.game_robot_status.chassis_power_limit == 60)
			{
			 if(rush_flag==0)
       for(int i=0 ; i < 4 ; i++)
				{
				if(chassis->Vectors.SpeedNo[i]>5336)
          {
					   chassis->Vectors.SpeedNo[i]=5336;
					}
				if(chassis->Vectors.SpeedNo[i]<-5336)
          {
					   chassis->Vectors.SpeedNo[i]=-5336;
					}	
				}
			}
			if(referee2022.game_robot_status.chassis_power_limit == 80)
			{
			 if(rush_flag==0)
       for(int i=0 ; i < 4 ; i++)
				{
				if(chassis->Vectors.SpeedNo[i]>4069)
          {
					   chassis->Vectors.SpeedNo[i]=4069;
					}
				if(chassis->Vectors.SpeedNo[i]<-4069)
          {
					   chassis->Vectors.SpeedNo[i]=-4069;
					}	
				}
			}
			if(referee2022.game_robot_status.chassis_power_limit == 100)
			{
			 if(rush_flag==0)
       for(int i=0 ; i < 4 ; i++)
				{
				if(chassis->Vectors.SpeedNo[i]>7036)
          {
					   chassis->Vectors.SpeedNo[i]=7036;
					}
				if(chassis->Vectors.SpeedNo[i]<-7036)
          {
					   chassis->Vectors.SpeedNo[i]=-7036;
					}	
				}
			}
*/
	    if(rush_flag==0)
			{
  		if(referee2022.game_robot_status.chassis_power_limit == 50)
			{
				if(chassis->Movement.Vx>1213){
		    chassis->Movement.Vx=1213;
		    }
		    if(chassis->Movement.Vx<-1213){
		    chassis->Movement.Vx=-1213;
		    }				
		    if(chassis->Movement.Vy>1213){
		    chassis->Movement.Vy=1213;
		    }
		    if(chassis->Movement.Vy<-1213){
		    chassis->Movement.Vy=-1213;
		    }	
			}
			if(referee2022.game_robot_status.chassis_power_limit == 60)
			{
				if(chassis->Movement.Vx>1952){
		    chassis->Movement.Vx=1952;
		    }
		    if(chassis->Movement.Vx<-1952){
		    chassis->Movement.Vx=-1952;
		    }				
		    if(chassis->Movement.Vy>1952){
		    chassis->Movement.Vy=1952;
		    }
		    if(chassis->Movement.Vy<-1952){
		    chassis->Movement.Vy=-1952;
			  }
		  }
			if(referee2022.game_robot_status.chassis_power_limit == 80)
			{
				if(chassis->Movement.Vx>3512){
		    chassis->Movement.Vx=3512;
		    }
		    if(chassis->Movement.Vx<-3512){
		    chassis->Movement.Vx=-3512;
		    }				
		    if(chassis->Movement.Vy>3512){
		    chassis->Movement.Vy=3512;
		    }
		    if(chassis->Movement.Vy<-3512){
		    chassis->Movement.Vy=-3512;
			  }
			}
			if(referee2022.game_robot_status.chassis_power_limit == 100)  
			{
				if(chassis->Movement.Vx>4152){
		    chassis->Movement.Vx=4152;
		    }
		    if(chassis->Movement.Vx<-4152){
		    chassis->Movement.Vx=-4152;
		    }				
		    if(chassis->Movement.Vy>4152){
		    chassis->Movement.Vy=4152;
		    }
		    if(chassis->Movement.Vy<-4152){
		    chassis->Movement.Vy=-4152;
			  }
			}
			}		
		}			
		  SwerveChassisMotionControl(&swerveChassis,&rc_Ctrl , &Holder); 
	}
}

