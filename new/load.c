#include "load.h"
#include "init.h"
#include "dr16.h"
#include "fdcan.h"
#include "driver_timer.h"
#include "heat_control.h"
#include "referee.h"
#include "hardware_config.h"

#include "load.h"
#include "init.h"
#include "dr16.h"
#include "fdcan.h"
#include "driver_timer.h"
#include "heat_control.h"
#include "referee.h"
#include "hardware_config.h"

uint16_t Friction_Start_Cnt;
uint16_t Friction_Stop_Cnt;
uint16_t  OPEN_Compare =2100;  //���ոǿ�
uint16_t  CLOSE_Compare = 1150; //���ոǹ�
uint16_t heat_t=28;
float shoot_sense =0.205f;//0.1708f;//0.220517f;  //0.255f;
float Finish_Angle=32.0f;
 /**
  * @brief ���������ʼ��
  */
void LoadInit(Shoot_Info* shoot, BasePID_Object friction_pid, BasePID_Object load_pid, CanNumber canx)        
{
	
	MotorInit(&shoot->Motors3508.motor[0], 0 , Motor3508, canx, 0x201);
	MotorInit(&shoot->Motors3508.motor[1], 0 , Motor3508, canx, 0x202);
	MotorInit(&shoot->Motors2006.motor[0], 0 , Motor2006, canx, 0x203);
	BasePID_Init(&shoot->Motors3508.RunPID[0],friction_pid.Kp,friction_pid.Ki,friction_pid.Kd,friction_pid.KiPartDetachment);
	BasePID_Init(&shoot->Motors3508.RunPID[1],friction_pid.Kp,friction_pid.Ki,friction_pid.Kd,friction_pid.KiPartDetachment);
	BasePID_Init(&shoot->Motors2006.RunPID[0],load_pid.Kp,load_pid.Ki,load_pid.Kd,load_pid.KiPartDetachment);
	BasePID_Init(&shoot->Motors2006.RunPID[1],600,0,7,0);
	shoot->Forward_Reverse_Flag=1;
	shoot->Shoot_Mode = 0;
	shoot->Shoot_Target_Num = 1;
	shoot->Target_Firing_Rate = 6000;  //8100
	shoot->First_Single_Flag = 0;
	shoot->Finish_Flag = 0;
	shoot->Shoot_Gap = 0 ;
	shoot->Shoot_sum_cut = 0 ;
	shoot->Shoot_Gap_Limit = 0;
	shoot->Shoot_Target_Sum = 1;
	shoot->Friction_Status_Flag=1
	;
}


float delta_angle;
uint16_t Friction_Three_Speed_Left[3]={4000,0,0};//6000

uint16_t Friction_Three_Speed_Right[3]={3940,0,0};//5962

uint16_t Friction_Speed_Left  =4695;  //6995  //4695
uint16_t Friction_Speed_Right =4700;  //7000  //4700
uint16_t Friction_Dafu_Left  =7395;
uint16_t Friction_Dafu_Right =7400;
uint16_t Firing_Rate=4300;

/**
  * @brief ��������ٶ�(ɵ�Ͽ��Ʒ�)
  */
void Load_Motor_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,float Target_Angle, float Feedback_Angle)
{	
//	//���������ת��Ϊ�Ƕ�Load_Info.speed_raw * 2�У����Ǹõ��Ϊ1/36���ٵ����������Ҫ/36��
//	//��������������ֵ�����磺ʵ��תһȦ��348����˴˴���Ҫ*(360/348)���˴�������������Ե���
//	delta_angle =shoot->Motors2006.motor[0].Data.SpeedRPM  * 0.001f*1.19f ;//0.16890283083816092679906684856341 * 0.80f
//	//if((delta_angle<0.01)&&(delta_angle>-0.01)) delta_angle=0;
//	if(abs(delta_angle)<0.3f) delta_angle=0;//��ֹ�ܷ�
//	/*���˲������ԽǶ�*/
//	booster->Angle_Plate += delta_angle;//����6��һ��ת���ĽǶ�
	if ((Target_Angle - Feedback_Angle) < Finish_Angle && (Target_Angle - Feedback_Angle) > Finish_Angle*(-1))
  {
		booster->Target_Speed[2] = 0;
		
		shoot->Finish_Flag=1;

	}
	else
	{
		shoot->Finish_Flag=0;
		
  	booster->Target_Speed[2]=	shoot->Forward_Reverse_Flag * shoot->Target_Firing_Rate;
		 
  	if(shoot->Forward_Reverse_Flag==1)	
	 { 
		 //�������������� 

		  if(shoot->Motors2006.motor[0].Data.SpeedRPM<50) //��ת��������
		  {
			  booster->Inversion_Cnt++;
		 	  if(booster->Inversion_Cnt>300)
		  	{
				 booster->Target_Speed[2]=-3000;
				 if(booster->Inversion_Cnt>300+216000.0f/abs((int)booster->Target_Speed[2]))
				 {	
					booster->Inversion_Cnt=0;
				 }
			  }
		  }
 
	 } 
  else if(shoot->Forward_Reverse_Flag==-1)
	 {

			if(shoot->Motors2006.motor[0].Data.SpeedRPM>-50) //��ת��������
		 {
			booster->Inversion_Cnt++;
			if(booster->Inversion_Cnt>300)
			{
				booster->Target_Speed[2]=3000;
				if(booster->Inversion_Cnt>300+216000.0f/abs((int)booster->Target_Speed[2]))
				{	
					booster->Inversion_Cnt=0;
				}
			}
		 }	
//	  }		 
	 }
  }
}

/**
  * @brief �����̿���
  */
void Load_Shoot_control(Shoot_Info* shoot ,Friction_Load_Info* booster,RC_Ctrl* rc_ctrl)
{
	if(shoot->Shoot_Mode==1)//����ģʽ //δ��
	{
		booster->Angle_Target=shoot->Forward_Reverse_Flag*1000000;
	  shoot->First_Single_Flag=0;
	}
	else
	{
		if(shoot->First_Single_Flag<1)
		{
		booster->Angle_Target=0;
		booster->Angle_Target_Last=0;
    booster->Angle_Plate=shoot->Forward_Reverse_Flag*(0);//��ʼλ��   					
		}
		shoot->First_Single_Flag++;
		booster->Target_Angle =rc_ctrl->OneShoot  *45;
		booster->ThreeAngle = rc_ctrl->ThreeShoot * 45;
		if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)
		{
	   booster->Angle =shoot->Shoot_Target_Sum *45;; 
		}
		if(((shoot->Crowd_back_Flag==0)&&(shoot->Stop_Back_Flag==0)))
		booster->Angle_Target = booster->Target_Angle + booster->Angle + booster->ThreeAngle;
	  if((shoot->Stop_Back_Flag==1))//||((rc_ctrl->rc.s1==3)&&(rc_ctrl->mouse.press_l_flag==0)&&(rc_ctrl->key_Q_flag==0)))
		{
		booster->Angle_Target = booster->Angle_Plate;
		shoot->Stop_Back_Flag=0;
		}
	  if(shoot->Crowd_back_Flag==1)
		{
		booster->Angle_Target = booster->Angle_Plate-45;
		shoot->Crowd_back_Flag=0;
		}
		if(booster->Angle_Target_Last != booster->Angle_Target)
		{
			ShootModeToBrain=1;
			shoot->Finish_Flag=0;
		}
		if(abs(booster->Angle_Target - booster->Angle_Plate)<5) //���С��5����Ϊ��ɴ�
		{
			ShootModeToBrain=0;
			shoot->Finish_Flag=1;
		}
		booster->Angle_Target_Last = booster->Angle_Target;
			if(shoot->Forward_Reverse_Flag==1)	        
	 { 
		 //�������������� 
		  if((shoot->Motors2006.motor[0].Data.SpeedRPM<50)&&(ShootModeToBrain==1)) //��ת��������
		  {
				booster->Inversion_Cnt++;
		 	  if(booster->Inversion_Cnt==300)
//				rc_ctrl->OneShoot--;
				shoot->Crowd_back_Flag=1;
				if(booster->Inversion_Cnt>300)
				booster->Inversion_Cnt=301;
			}
			else
				booster->Inversion_Cnt=0;
		}
		shoot->Shoot_Target_Num=0;
    if(shoot->Finish_Flag==1)
		{
		 booster->Inversion_Cnt=0;
		}
	}			
}
/*�����Ƿ��ǹ���������� */

/**
  * @brief  Ħ���ֿ���
  *         �����ıջ� �úõ� ���ٷ�����ʱ
  */

void Friction_Load_Fire_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,Trace* Info_Vision)
{  	
	 uint8_t i;
							/*���ģʽ*/
	
   Heat_Control();//����ȴ
	 
	//Q�� ���� ctrl + Q ������ 
	if(shoot->Friction_Status_Flag==1&&tim14.HolderTime > 500&&rc_Ctrl.isOnline == 1)
	{
//		if(referee2022.game_robot_status.mains_power_shooter_output==1)
//			{	
			Friction_Start_Cnt++;	
		  Friction_Stop_Cnt=0;
		/*���ٴ���*/ //ʼ���Ե�ǰ������� ����������
        if(Friction_Start_Cnt>1000)		//1s������
	      {
	       	Friction_Start_Cnt=1000;
	      }
//			}
//		if(referee2022.game_robot_status.mains_power_shooter_output==0)
//		{
//	   	Friction_Start_Cnt=0;
//		}

//����
//		Friction_Three_Speed_Left[0]=4750;//5100
//		Friction_Three_Speed_Right[0]=4770;//5060
		if(Info_Vision->Hit_Mode!=1)//15 0x0f
	{
		Friction_Three_Speed_Left[0]  = Friction_Speed_Left;//7000;//4800; //5200;//8400;//4370;//4500 �����ᳬ0.��
		Friction_Three_Speed_Right[0] = Friction_Speed_Right;// 7000;//4800; //5200;//8400;//4370;//4500
	}

	else if(Info_Vision->Hit_Mode==1)//����
	{
		Friction_Three_Speed_Left[0]=Friction_Dafu_Left;//7680
		Friction_Three_Speed_Right[0]=Friction_Dafu_Right;//7000;//7680

	}
	else
	{
		Friction_Three_Speed_Left[0]=0;
		Friction_Three_Speed_Right[0]=0;		
	}
//ȫ����
//		Friction_Three_Speed_Left[0]=4800;//5100
//		Friction_Three_Speed_Right[0]=4780;//5060	
//		Friction_Three_Speed_Left[0]=5100;//5100
//		Friction_Three_Speed_Right[0]=5060;//5060
		
	booster->Target_Speed[0]=Friction_Three_Speed_Left[0]*Friction_Start_Cnt/1000;
	booster->Target_Speed[1]=-Friction_Three_Speed_Right[0]*Friction_Start_Cnt/1000;

	//	��һ��������
	//	if(Muzzle.on_time_heat>=Get_Max_Heat(game_robot_state.robot_level)-25) 
//		shoot->Target_Firing_Rate=4320;
	if(shoot->SuperHeatMode ==0){
	if(Muzzle.on_time_heat>=referee2022.game_robot_status.shooter_id1_17mm_cooling_limit-heat_t)
	{
	 shoot->Target_Firing_Rate=0;
		Muzzle.heat_status=0; //��������
  }
	else 
	{ 
		shoot->Target_Firing_Rate=Firing_Rate;
		Muzzle.heat_status=1; //��������
	}
}	
	else if(shoot->SuperHeatMode ==1){
	shoot->Target_Firing_Rate=Firing_Rate;
	}
//	  shoot.Target_Firing_Rate=4320;
	  Load_Shoot_control(&Shoot,&Booster,&rc_Ctrl);	
		
	//���������ת��Ϊ�Ƕ�Load_Info.speed_raw * 2�У����Ǹõ��Ϊ1/36���ٵ����������Ҫ/36��
	//��������������ֵ�����磺ʵ��תһȦ��348����˴˴���Ҫ*(360/348)���˴�������������Ե���
	delta_angle =shoot->Motors2006.motor[0].Data.SpeedRPM  * 0.001f*shoot_sense ;//0.16890283083816092679906684856341 * 0.80f
	//if((delta_angle<0.01)&&(delta_angle>-0.01)) delta_angle=0;
	if(abs(delta_angle)<0.01f) delta_angle=0;//��ֹ�ܷ�
	/*���˲������ԽǶ�*/
	booster->Angle_Plate += delta_angle;//����6��һ��ת���ĽǶ�
	
  Load_Motor_Control(&Shoot,&Booster,booster->Angle_Target,booster->Angle_Plate);

	/*Ħ���ֻ�����*/
		booster->Current_1[0]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+0),shoot->Motors3508.motor[0].Data.SpeedRPM,booster->Target_Speed[0],0);
		booster->Current_1[1]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+1),shoot->Motors3508.motor[1].Data.SpeedRPM,booster->Target_Speed[1],1);
	if(Vision_Info.Hit_Mode!=1)
    booster->Current_1[2]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors2006.RunPID+0),shoot->Motors2006.motor[0].Data.SpeedRPM,booster->Target_Speed[2],2);	
  else if(Vision_Info.Hit_Mode==1)
		booster->Current_1[2] = BasePID_AngleControl(&shoot->Motors2006.RunPID[1], booster->Angle_Target, booster->Angle_Plate, shoot->Motors2006.motor[0].Data.SpeedRPM );
	if( booster->Current_1[2] >9000)  booster->Current_1[2] =9000;
	if( booster->Current_1[2] <-9000)  booster->Current_1[2] =-9000;
	}
	else if(rc_Ctrl.isOnline == 0||shoot->Friction_Status_Flag==0)
	{  
	  	Friction_Start_Cnt=0;
	if((Shoot.Motors3508.motor[0].Data.SpeedRPM!=0)||(Shoot.Motors3508.motor[1].Data.SpeedRPM!=0))
	{
		Friction_Stop_Cnt += Friction_Three_Speed_Left[0]*0.0003;
		if(Friction_Stop_Cnt >= Friction_Three_Speed_Left[0])
			Friction_Stop_Cnt = Friction_Three_Speed_Left[0];
		booster->Target_Speed[0]=Friction_Three_Speed_Left[0]-Friction_Stop_Cnt;
		booster->Target_Speed[1]=booster->Target_Speed[0];
		booster->Current_1[0]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+0),shoot->Motors3508.motor[0].Data.SpeedRPM,booster->Target_Speed[0],0);
		booster->Current_1[1]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+1),shoot->Motors3508.motor[1].Data.SpeedRPM,booster->Target_Speed[1],1);
	}
	else
			for(i=0;i<3;i++)//��Ħ���� ������
			{        					
  	  	booster->Current_1[i]=0;
			}	
	}
     MotorFillData(&shoot->Motors3508.motor[0], booster->Current_1[0]);
		 MotorFillData(&shoot->Motors3508.motor[1], booster->Current_1[1]);
		 MotorFillData(&shoot->Motors2006.motor[0], booster->Current_1[2]);
}

/**
  * @brief ���ոǶ������
  */
 void Steering_Engine_Control(RC_Ctrl* rc_ctrl,Shoot_Info* shoot)
{
	if(shoot->BulletCap_Status_Flag==1)
	{
		Steering_Engine_Setcompare(OPEN_Compare);//CLOSE_Compare
		//__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_3,2100);	
	}
	else if(shoot->BulletCap_Status_Flag==0) 
	{
		//__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_3,1000);
		Steering_Engine_Setcompare(CLOSE_Compare);//OPEN_Compare
	}	
}

/**
  * @brief ���ոǶ������
  */
void Steering_Engine_Setcompare(uint16_t Compare)
{
		TIM3->CCR4=Compare;	
}

/**
  * @brief �������
  */
void Shoot_Fire_Mode_Control(RC_Ctrl* rc_ctrl,Shoot_Info* shoot)
{ //�Զ���
//	if(rc_ctrl->rc.s1==2&&rc_ctrl->rc.s1_last==3)
//	{
//		shoot->Shoot_Target_Num=1;
//		shoot->Shoot_Target_Sum++;
//		shoot->Shoot_Mode=0;
//	}
//	if((RC_Ctl.key_Q_flag==1&&RC_Ctl.key_ctrl_flag==0&&RC_Ctl.key_E_flag==0&&RC_Ctl.key_X_flag==0)||(Vision_Info.Flag_Shoot_B_Enable==1&&Vision_Info.Flag_Shoot_B_Enable_Last==0))//

//	if(RC_Ctl.mouse.press_l_flag==1||RC_Ctl.rc.s1==1||Vision_Info.Flag_Auto_Enable==1)//���ٵ���||Vision_Info.Flag_Auto_Enable==1
	if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)//���ٵ���||Vision_Info.Flag_Auto_Enable==1		
	{
		shoot->Shoot_Mode=0;
		shoot->Shoot_Gap++;
		shoot->Shoot_Gap_Limit=0;
		if((rc_Ctrl.isOnline == 1)&&(shoot->Shoot_Gap>shoot->Shoot_Gap_Limit))   //���䳬����ֵʱת��ģʽΪ����
		{
			shoot->Shoot_Gap=0;
			shoot->Shoot_Target_Num=1;
			shoot->Shoot_sum_cut ++;  //����Ƶ��200֡
			if((Muzzle.heat_status==1)&&(shoot->Shoot_sum_cut%50==0)&&(shoot->Crowd_back_Flag==0))  //��Ƶ��һ��20��
				shoot->Shoot_Target_Sum++;
			else if(Muzzle.heat_status==0)
				shoot->Stop_Back_Flag=1;
		}
	} 
//	if(rc_ctrl->key_ctrl_flag==1&&rc_ctrl->key_X_flag==1)//����Ħ����
//  { 
//	 if(shoot->Friction_Status_Flag==1)
//	 {
//	 shoot->Friction_Status_Flag=0;    //��
//	 }
//	 else if(shoot->Friction_Status_Flag==0)	
//	 {
//	 shoot->Friction_Status_Flag=1; 		//��
//	 }		 
//  }	

	}  
	
/**
  * @brief ��������������� 
  */
void Friction_Shoot_Heat_Control_Deal(RC_Ctrl* rc_ctrl)
{	
 Shoot_Fire_Mode_Control(&rc_Ctrl,&Shoot);

 if(rc_ctrl->key_V_flag==1&&rc_ctrl->key_ctrl_flag!=1)
 {
//	 Holder.Pitch.Target_Angle=0;
//	 Muzzle.over_heat=0;  //�رճ�����
   rc_ctrl->key_E_flag=0; //�ص��ո�
//	 RC_Ctl.key_Z_flag=0; //
//	 Chassis.Roll_Flag=0;	//�ر�����
//	 Chassis.Rock_Flag=0; //�ر�ҡ��
	 rc_ctrl->key_R_flag=0;	//�ر�����
	 rc_ctrl->key_C_flag=0; //�ر�ҡ��
//	 shoot.Friction_Status_Flag=1;//����Ħ����
//	 Vision_Info.Hit_Mode=0;//�رմ��
 }
//	if(RC_Ctl.key_V_flag==1&&RC_Ctl.key_ctrl_flag==1)
//	 IWDG_Flag = 0;   //���Ź�ȡ��ι��

}




/**
  * @brief 
  */
float Friction_Load_Pid_Control(BasePID_Object* base_pid,float Feedback,int16_t target,uint8_t who)
{   
	
	static  float  E_Sum[3];

	if(who==2)
	{
	base_pid->Error = target - Feedback;//Wheel_v_h_speed[who];
	
  base_pid->KpPart = base_pid->Error * base_pid->Kp;

	E_Sum[who]+=base_pid->Error*base_pid->Ki;
	
	if(target==0) E_Sum[who]=0;	
	
	base_pid->KiPart=E_Sum[who];
	
	if(base_pid->KiPart>1200) base_pid->KiPart=1000;
	if(base_pid->KiPart<-1200) base_pid->KiPart=-1000;
		
	base_pid->Out=base_pid->KpPart+base_pid->KiPart; 
	
	if(base_pid->Out>10000) base_pid->Out=10000;//����ֵ����̫��
	if(base_pid->Out<-10000) base_pid->Out=-10000;
	return base_pid->Out;
  }
	if(who<2)
	{
	base_pid->Error = target - Feedback;
		
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
 
	if(base_pid->Error<50||base_pid->Error>-50)//�������� 
	E_Sum[who]+=base_pid->Error*base_pid->Ki;
	
	if(target==0) E_Sum[who]=0;	
	base_pid->KiPart=E_Sum[who];
	
	if(base_pid->KiPart>1000) base_pid->KiPart=1000;
	if(base_pid->KiPart<-1000) base_pid->KiPart=-1000;
	
	base_pid->Out=base_pid->KpPart+base_pid->KiPart;
	
	if(base_pid->Out>30000)base_pid->Out=30000;
	if(base_pid->Out<-30000)base_pid->Out=-30000;
  return  base_pid->Out;		
  }
	return 0;

}

