#include "check.h"
#include "referee.h"

Check_Robot_State check_robot_state;

/**
  *��ȡ������״̬���������Ƿ����ߣ�
	*/
	
void RobotOnlineState(Check_Robot_State *CheckRobotState, Referee2022 *referee2022)
{
	CheckRobotState->usart_state.Check_receiver++;
	CheckRobotState->usart_state.Check_vision++;
	CheckRobotState->usart_state.Check_referee++;
	
//	CheckRobotState->motor6020_state.Check_can2_0x205++;
//	CheckRobotState->motor6020_state.Check_can2_0x206++;
//	CheckRobotState->motor6020_state.Check_can2_0x207++;
//	CheckRobotState->motor6020_state.Check_can2_0x208++;
//	CheckRobotState->motor3508_state.Check_can2_0x201++;
//	CheckRobotState->motor3508_state.Check_can2_0x202++;
//	CheckRobotState->motor3508_state.Check_can2_0x203++;
//	CheckRobotState->motor3508_state.Check_can2_0x204++;
//	
//	CheckRobotState->motor6020_state.Check_can1_0x205++;                                                               //ϣ������ÿ������Ƿ����ߣ����ܺ������йأ���ʱ�������ײ㣬��ѧ�������Ժ������Լ���
//	CheckRobotState->motor6020_state.Check_can1_0x206++;
//  CheckRobotState->motor3508_state.Check_can1_0x201++;
//	CheckRobotState->motor3508_state.Check_can1_0x202++;
//	CheckRobotState->motor2006_state.Check_can1_0x203++;
	
	CheckRobotState->referee_state.heat          = referee2022->power_heat_data.shooter_id1_17mm_cooling_heat;           //ǹ�ڵ�ǰ����
	CheckRobotState->referee_state.heat_limit    = referee2022->game_robot_status.shooter_id1_17mm_cooling_limit;        //ǹ����������
	CheckRobotState->referee_state.power         = referee2022->power_heat_data.chassis_power;                           //���̵�ǰ����
	CheckRobotState->referee_state.power_limit   = referee2022->game_robot_status.chassis_power_limit;                   //���̹�������
	CheckRobotState->referee_state.power_buffer  = referee2022->power_heat_data.chassis_power_buffer;                    //���̵�ǰ��������
//	CheckRobotState->referee_state.power_buffer_limit = referee2022->game_robot_status.                                //���̻����������ƣ���δ������

	if(CheckRobotState->usart_state.Check_receiver > 100)
		CheckRobotState->usart_state.Check_receiver = 100;                                                                 //�޷���������ֵ�ӵ�2^16��ͻ���0��ʼ���𣬳������ʧ��
}