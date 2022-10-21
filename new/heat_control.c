#include "heat_control.h"
#include "referee.h"
 Heat_Info Muzzle=
 {
	 .cooling_times=1
 };
 /*1msִ��1��*/
 /*
 ����ϵͳ��Ѫ�߼�
     �趨�����˵�ǹ����������Ϊ Q0����ǰǹ������Ϊ Q1�� ����ϵͳÿ��⵽һ�� 17mm ���裬 ��ǰǹ����
 �� Q1 ���� 10���� 17mm ����ĳ��ٶ��޹أ���
			�� Q1 > Q0���û����˶�Ӧ�����ֵ��Եĵ�һ�ӽǿ��ӶȽ��͡�ֱ�� Q1 < Q0����һ�ӽǲŻ�ָ���
 ��
      �� 2Q0 > Q1 > Q0��ÿ 100 ms �۳�Ѫ�� = ((Q1 - Q0) / 250) / 10 * ����Ѫ��
			�� Q1 �� 2Q0�����̿۳�Ѫ�� = (Q1 - 2Q0) / 250 * ����Ѫ������Ѫ���� Q1 = 2Q0��
 */
 
void Heat_Control()
{ 
//	if(referee2022.power_heat_data.renewshooter_heat0==0)//����ϵͳ���������£�������ȴ
//	{		
//			//Muzzle.on_time_heat-=Get_Heat_Cold_Speed(game_robot_state.robot_level)*Muzzle.cooling_times/1000.0f;//
//	  	Muzzle.on_time_heat-=referee2022.game_robot_status.shooter_id1_17mm_cooling_rate*Muzzle.cooling_times/1000.0f; //ִ������Ϊ1kHz
//		
//	}
	 if(referee2022.power_heat_data.renewshooter_heat0==1)// ����ϵͳ������Ϣ������ ���¸���ǰ������ֵ
	{
		  Muzzle.on_time_heat=referee2022.power_heat_data.shooter_id1_17mm_cooling_heat;//����ϵͳ��������
//		  referee2022.power_heat_data.shooter_heat0=0;  //δ����
	}
	
	if(Muzzle.on_time_heat<0)
	{
		 Muzzle.on_time_heat=0;  //
	}
	if(Muzzle.over_heat==1)//������
	{
		Muzzle.on_time_heat=0;
	}
}
uint16_t Get_Max_Heat(uint8_t Robot_Level)//��ȡǹ����������
{
	if      (Robot_Level==1)    return 120;	//
	else if (Robot_Level==2)    return 240;				
	else if (Robot_Level==3)    return 360;	

	//�Ҳ������
	
	return referee2022.power_heat_data.shooter_id1_17mm_cooling_heat;
}
uint16_t Get_Heat_Cold_Speed(uint8_t Robot_Level)//��ȡǹ��������ȴ�ٶȣ������ӵ��ٶ�����Ϊ30m/s��
{
	if      (Robot_Level==1)    return 20;	//ǹ��ÿ����ȴ 30 40 50	Ѫ������20%����ȴ�ٶ�x2
	else if (Robot_Level==2)    return 40;				
	else if (Robot_Level==3)    return 60;	
	return 0;
}