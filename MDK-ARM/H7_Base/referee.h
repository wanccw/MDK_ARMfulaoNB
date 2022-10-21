
#ifndef __REFEREE_H
#define __REFEREE_H

#include "main.h"

#define data_addr 7
#define max_single_pack_len 50
#define packs 10
#define frame_header_len 5 
#define pack_len 7+referee2022.frame_info.head.data_len+2
#define BSP_USART3_DMA_RX_BUF_LEN	256 

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
typedef struct
{
	struct
	{
	struct
	{
		uint8_t sof;
		uint16_t data_len;
		uint8_t seq;
		uint8_t crc8;
		}head;
	uint16_t cmd_id;
	uint8_t  frame_tail[2];
  }frame_info;

/*1. ����״̬���ݣ� 0x0001�� ����Ƶ�ʣ� 1Hz*/
struct  
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} game_status;

/*2. ����������ݣ� 0x0002�� ����Ƶ�ʣ�������������*/
 struct
{
	uint8_t winner;
} game_result;
/*3. ������Ѫ�����ݣ� 0x0003������Ƶ�ʣ� 1Hz*/
 struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} game_robot_hp;
/*4. ���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л�����*/
struct
{
	uint8_t dart_belong;   //������ڶ��� 1���췽���� 2����������
	uint16_t stage_remaining_time;//����ʱ��ʣ�����ʱ�� ��λ s
}  dart_state;

/*5. �˹�������ս���ӳ���ͷ���״̬��0x0005������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����л����ˡ�*/
 struct
{
	uint8_t F1_zone_status:1;
	uint8_t F1_zone_buff_debuff_status:3;
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;
} ext_ICRA_buff_debuff_zone_status_t;


/*6.�����¼����ݣ�0x0101������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�*/
 struct
{
	uint32_t event_type;
} event_data;

/*7. ����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�
*/
 struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} supply_projectile_action;

/*8. ���о�����Ϣ�� cmd_id (0x0104)������Ƶ�ʣ����淢������*/
struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} referee_warning;

/*9. ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�*/
 struct
{
	uint8_t dart_remaining_time;
}  dart_remaining_time;

/*10. ����������״̬�� 0x0201�� ����Ƶ�ʣ� 10Hz�����ͷ�Χ����һ�����ˡ�*/
 struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_heat0_cooling_rate;
	uint16_t shooter_heat0_cooling_limit;
	uint16_t shooter_heat1_cooling_rate;
	uint16_t shooter_heat1_cooling_limit;
	uint16_t chassis_power_limit;
	uint8_t shooter_heat0_speed_limit;
	uint8_t shooter_heat1_speed_limit;
	uint8_t max_chassis_power;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
} game_robot_status;

/*11. ʵʱ�����������ݣ� 0x0202�� ����Ƶ�ʣ� 50Hz�����ͷ�Χ����һ�����ˡ�
*/
 struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_heat0; 
	uint16_t shooter_heat1; 
	uint16_t mobile_shooter_heat2;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint8_t renewchassis_power;
	uint8_t renewchassis_power_buffer;
	uint8_t renewshooter_heat0;
} power_heat_data;

/*12. ������λ�ã� 0x0203�� ����Ƶ�ʣ� 10Hz�����ͷ�Χ����һ�����ˡ�
*/
struct
{
	float x;
	float y;
	float z;
	float yaw;
} game_robot_pos;

/*13. ���������棺 0x0204������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ����һ�����ˡ�
*/
 struct
{
	uint8_t power_rune_buff;
}buff;

/*14. ���л���������״̬�� 0x0205�� ����Ƶ�ʣ� 10Hz
*/
 struct
{
	uint16_t energy_point;
	uint8_t attack_time;
} ext_aerial_robot_energy_t;


/*15. �˺�״̬�� 0x0206�� ����Ƶ�ʣ��˺���������
*/
 struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} robot_hurt;

/*16. ʵʱ�����Ϣ�� 0x0207�� ����Ƶ�ʣ��������*/
 struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} shoot_data;

//17. �ӵ�ʣ�෢������ 0x0208������Ƶ�ʣ� 1Hz ���ڷ��ͣ� ���л������Լ��ڱ����������ط���

struct
{
  uint16_t bullet_remaining_num;
} bullet_remaining;


//18.������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�

 struct
{
	uint32_t rfid_status;
}  rfid_status;

//19. ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
 struct
{
	uint8_t dart_launch_opening_status; //���ڷ����״̬
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
	uint16_t operate_launch_cmd_time;
} dart_client_cmd;

//�����˼佻������

//1. �������ݽ�����Ϣ��0x0301��
 struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
}	robot_interactive_data;

 struct //0x0301 �ͻ�������id 0xd180
{
	uint8_t data[30];
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data;

}Referee2022;

extern Referee2022 referee2022;


extern uint8_t meta_data[BSP_USART3_DMA_RX_BUF_LEN]__attribute__((at(0x24028000)));

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Referee_Data_Diapcak(uint8_t *data,uint8_t this_time_len);
uint8_t Referee_callback(uint8_t * recBuffer, uint16_t len);
#endif