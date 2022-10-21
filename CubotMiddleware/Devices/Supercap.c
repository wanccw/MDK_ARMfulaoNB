#include "Supercap.h"
#include "fdcan.h"
#include "driver_can.h"
#include "dr16.h"
#include "motor.h"
#include "referee.h"
#include "hardware_config.h"
CAN_TxBuffer txBuffer0x6FFforCAN1={
	.Identifier = 0x6ff
};


uint8_t Supercap_rxCallBack(CAN_Object canx, CAN_RxBuffer rxBuffer ,Supercap* Cap)
{
	if(rxBuffer.Header.Identifier == 0x601)
	{
	Cap->cap_state.Voltage = ((rxBuffer.Data[0]<<8)+rxBuffer.Data[1])*0.01f;
	Cap->cap_state.Current = ((rxBuffer.Data[2]<<8)+rxBuffer.Data[3]);
	}
}

void SupercapControl(CAN_Object can, RC_Ctrl* rc_ctrl , Supercap* Cap)
{
	if(Cap->cap_state.Supercap_Mode == 1 && Cap->cap_state.Voltage > 16.2)
	{
		txBuffer0x6FFforCAN1.Data[0] = 1;
		Cap->cap_state.Supercap_Flag = 1;
	}
	else
	{
		txBuffer0x6FFforCAN1.Data[0] = 0;
		Cap->cap_state.Supercap_Flag = 0;
	}
	if(Cap->cap_state.Supercap_Charge_mode==0)
		Cap->cap_state.Supercap_Charge=0;
	else if (Cap->cap_state.Supercap_Charge_mode==1)
		Cap->cap_state.Supercap_Charge=1;
	txBuffer0x6FFforCAN1.Data[1] = referee2022.game_robot_status.chassis_power_limit;
	txBuffer0x6FFforCAN1.Data[2] = Cap->cap_state.Supercap_Charge;
	txBuffer0x6FFforCAN1.Data[3] = 0;
	txBuffer0x6FFforCAN1.Data[4] = 0;
	txBuffer0x6FFforCAN1.Data[5] = 0;
	txBuffer0x6FFforCAN1.Data[6] = 0;
	txBuffer0x6FFforCAN1.Data[7] = 0;
	
	CAN_Send(&can, &txBuffer0x6FFforCAN1);   
}
//#include "Super_Cap.h"
//#include "Remote.h"
//#include "Chassis_Ctrl.h"
//#include "referee_2021.h"

///*Sub Board Function*/

//Sup_Cap_t Super_Cap=
//{
//	.smode=2,
//};
//Supper_Cap_Info cap_my=
//{
//	0

//};


///******************************************************************
//��������Check_Key
//���ܣ���ⳬ�����ݿ���
//������RC_Ctl_t * RC,Sup_Cap_t * Sup
//����ֵ����
//��������Use_Super_Cap
//�ϼ�������Super_Cap_Ctrl
//�¼���������
//******************************************************************/
//void Check_Key(RC_Ctl_t * RC,Sup_Cap_t * Sup)
//{
//	if(RC->key.shift_flag==1)
//	{
//		if(Sup->Voltage>16)
//			Sup->Use_Super_Cap=1;
//		else
//			Sup->Use_Super_Cap=0;
//	}
//	else
//		Sup->Use_Super_Cap=0;
//}
///******************************************************************
//��������Charge_Ctrl
//���ܣ����ݳ�����
//������Sup_Cap_t * Sup,Hero_Chassis_t * Chassis
//����ֵ����
//��������Charge
//�ϼ�������Super_Cap_Ctrl
//�¼���������
//******************************************************************/
//void Charge_Ctrl(Sup_Cap_t * Sup,Hero_Chassis_t * Chassis)
//{
//	int i=0;
//	int16_t speed_sum;
//	
//	for(i=0;i<4;i++)
//	{
//		speed_sum+=abs(Chassis->wheel.Feedback_Speed[i]);
//	}
//	
//	if(Chassis->wheel.Target_Speed_Sum>6000*4&&speed_sum<2000*4)
//		Sup->Charge=1;
//	else
//		Sup->Charge=0;
//	
//	if(Chassis->Flag.Rock==1)
//		Sup->Charge=1;
//	if(Sup->Use_Super_Cap==1) 
//		Sup->Charge=0;
//	
//	
//	if(Sup->Use_Super_Cap==1)
//		Sup->smode=1;
//	else if(Sup->Charge==1)
//		Sup->smode=2;
//	else
//		Sup->smode=2;
//}
///******************************************************************
//��������Super_Cap_Ctrl
//���ܣ����ݿ���
//��������
//����ֵ����
//�����������ݿ�����Ϣ
//�ϼ�������Hero_Sys_Run
//�¼�������Check_Key��Charge_Ctrl
//******************************************************************/
//void Super_Cap_Ctrl(void)
//{
////	Charge_Ctrl(&Super_Cap,&Hero_Chassis);
////	if(Super_Cap.Voltage<17)
////		Super_Cap.Use_Super_Cap=0;
//	
//	if(Super_Cap.Use_Super_Cap==1&&Super_Cap.Voltage>=20)
//		Super_Cap.smode=1;
//	if(Super_Cap.Use_Super_Cap==0||Super_Cap.Voltage<16)
//		Super_Cap.smode=2;
//}
///******************************************************************
//��������Super_Cap

//�Զ����ص���

//�Զ����ڳ����ݵĳ�繦��
//******************************************************************/
//int16_t zxj_sup_50w_limit=0;
//int16_t zxj_sup_80w_limit=0;
//int16_t zxj_sup_100w_limit=0;
//int16_t zxj_sup_120w_limit=0;
//int16_t zxj_sup_300w_limit=0;



//void Super_Cap_zxj(void)
//{
//	
//	
//	
//	
//	
//		if(RC_Ctl.key.shift_flag==1)
//	{

//		if(cap_my.voltage>16)
//			cap_my.Use_Super_Cap=1;
//		else
//			cap_my.Use_Super_Cap=0;
//		
//		
//	}
//	else
//	{
//		cap_my.Use_Super_Cap=0;
//	}
//	int16_t sum=0;
//	int8_t i=0;
//	for(i=0;i<4;i++)
//	{
//		
//		   sum=sum+Hero_Chassis.wheel.Output[i];

//	}
//	
////>������Ҫ�Ĺ����Ǹ��ݵ��̵�������ȷ��  �������ݳ�繦��
///*
//	˼·������ʱ�䳣����һֱ����ʱ�䳣�����������ݳ��ʱ��
//	Ȼ�������һ��  ��һ�γ�ʼ��
//	
//	Ȼ����ݲ�ͬ��  �������ƣ���ͬ�ĵ�������ѡ��
//	�������ݵĳ��ʱ��
//	
//	
//	
//	
//	
//	*/
//	
////	if(game_robot_state.chassis_power_limit==0)
////	{i=0;
////	}
////	


////	
////	
////	
////	

//	
//	
//	
//	
//	

//}