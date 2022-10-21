#ifndef INTERACTION_H__
#define INTERACTION_H__
#include "stm32h7xx.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920


//< cmd_id
#define ROBOTS_INTERACTION_ID 0x0301  //< ����ͨ��������

//< data_cmd_id
#define DRAW_CHAR_ID 0x0110					  //< ���ַ�ͼ�ε�����������
#define DELETE_GRAPHIC 0x0100         //< ɾ��ͼ�ε�����������
#define DRAW_SINGLE_GRAPHIC 0x0101    //< ����һͼ�ε�����������
#define DRAW_DOUBLE_GRAPHIC 0x0102    //< ������ͼ�ε�����������
#define DRAW_FIVE_GRAPHIC 0x0103      //< ������ͼ�ε�����������
#define DRAW_SEVEN_GRAPHIC 0x0104      //< ������ͼ�ε�����������


//< robots' & clients' ID
#define RobotRedHero 1
#define RobotRedEngineer 2
#define RobotRedInfantryNO3 3
#define RobotRedInfantryNO4 4
#define RobotRedInfantryNO5 5
#define RobotRedAerial 6
#define RobotRedSentry 7
#define RobotRedRadar 9

#define ClientRedHero 0x0101
#define ClientRedEngineer 0x0102
#define ClientRedInfantryNO3 0x0103
#define ClientRedInfantryNO4 0x0104
#define ClientRedInfantryNO5 0x0105
#define ClientRedAerial 0x0106

#define RobotBlueHero 101
#define RobotBlueEngineer 102
#define RobotBlueInfantryNO3 103
#define RobotBlueInfantryNO4 104
#define RobotBlueInfantryNO5 105
#define RobotBlueAerial 106
#define RobotBlueSentry 107
#define RobotBlueRadar 109

#define ClientBlueHero 0x0165
#define ClientBlueEngineer 0x0166
#define ClientBlueInfantryNO3 0x0167
#define ClientBlueInfantryNO4 0x0168
#define ClientBlueInfantryNO5 0x0169
#define ClientBlueAerial 0x016A


#define HighVolt 25
#define LowVolt 0



typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
}delete_Graphic_Operate;  //ext_client_custom_graphic_delete_t��uint8_t operate_type
/*ͼ��ɾ������*/


//bit 0-2
typedef enum
{
	NONE   = 0,/*�ղ���*/
	ADD    = 1,/*����ͼ��*/
	MODIFY = 2,/*�޸�ͼ��*/
	DELETE = 3,/*ɾ��ͼ��*/
}Graphic_Operate;

typedef enum
{
	LINE      = 0,//ֱ��
	RECTANGLE = 1,//����
	CIRCLE    = 2,//��Բ
	OVAL      = 3,//��Բ
	ARC       = 4,//Բ��
	FLOAT     = 5,//������
	INT       = 6,//������
	CHAR      = 7 //�ַ�
}Graphic_Type;

typedef enum
{
	RED_BLUE  = 0,//������ɫ	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*�Ϻ�ɫ*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*��ɫ*/
	BLACK     = 7,
	WHITE     = 8
}Graphic_Color;


/*
 * ����Э��֡ͷ
 */
typedef __packed struct
{
	uint8_t SOF;			    //0xA5
	uint16_t data_length; //data �ֽ���
	uint8_t seq;				  //�����
	uint8_t CRC8;
}frame_header_t;


/*
 * ���ݶ�ͷ�ṹ�����������ݵ����ݡ�������id�ͽ�����id
 */
typedef __packed struct //��ͷ�ṹ��
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;


typedef __packed struct //0x100 ɾ��ͼ��
{
	uint8_t operate_tpye; 
	uint8_t layer; 
}ext_client_custom_graphic_delete_t;


/*
 * ���ƻ���ͼ�����ݵĽṹ��
 */
typedef __packed struct 
{ 
uint8_t graphic_name[3]; //ͼ������
uint32_t operate_tpye:3; //ͼ�β���				0 ��				1 ����				2	�޸�				3	ɾ��
uint32_t graphic_tpye:3; //ͼ������				0 ֱ��			1 ����				2 ��Բ				4	Բ��				5 ������				6 ������				7 �ַ�
uint32_t layer:4; //ͼ��
uint32_t color:4; //��ɫ
uint32_t start_angle:9;//��ʼ�Ƕ�
uint32_t end_angle:9;//��ֹ�Ƕ�
uint32_t width:10; //�߿�
uint32_t start_x:11; //��ʼX����
uint32_t start_y:11;//��ʼY����
uint32_t radius:10; //�뾶�������С
uint32_t end_x:11; //��ֹX����
uint32_t end_y:11; //��ֹY����
}graphic_data_struct_t;


typedef struct {

/*
 * UI���Ƶ�һͼ�ε������ṹ�壬��30���ֽ�
 */
 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	ext_client_custom_graphic_delete_t ext_client_custom_graphic_delete;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_graphic_delete;  



/*
 * UI���Ƶ�һ�ַ��������ṹ�壬��60���ֽ�
 */
 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(51-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�
	uint8_t data[30];																														//< ��30���ֽڣ�ֱ�ӽ��ַ���char* ʹ�� strcpy ���ƽ�������
	
	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
} ext_client_custom_character;           



/*
 * UI���Ƶ�һͼ�ε������ṹ�壬��30���ֽ�
 */
 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_float;  


 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_int;  

 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_line;  


 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_circle;  


 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_rectangle;  


 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_ellipe;  

 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct;																		//< ��15���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_arc;  

/*
 * UI��������ͼ�ε������ṹ�壬��45���ֽ�
 */
 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(36-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct[2];																	//< ��30���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_double_t;  


/*
 * UI��������ͼ�ε������ṹ�壬��90���ֽ�
 */
 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct[5];																	//< ��30���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_patterning;  


 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct[5];																	//< ��30���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_patterning_second;  

/*
 * UI�����߸�ͼ�ε������ṹ�壬��90���ֽ�
 */
 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct[7];																	//< ��30���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_patterning_third;  


 __packed struct
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ��6���ֽ�
	graphic_data_struct_t grapic_data_struct[7];																	//< ��30���ֽ�

	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
}ext_client_custom_graphic_patterning_fourth;  

/*
 * �ͻ��˽�����Ϣ
 */
 __packed struct 
{ 
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	uint16_t target_robot_ID;     //< ������ID��Ϣ��2���ֽ�
	float target_position_x;			//< ������Ϣ��4���ֽ�
	float target_position_y; 			//< ������Ϣ��4���ֽ�
	
	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];
} ext_client_map_command; 


 __packed struct
{
	//< frame_header(5-byte) ֡ͷ����
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ������
	uint16_t cmd_id;
	
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ��6���ֽ�
	uint8_t data[15]; 
	//< frame_tail(2-byte��CRC16������У��)
	uint8_t CRC16[2];	
} robot_interactive_data; 


}UI_t;



//��һ���õ�
typedef __packed struct {
	uint8_t camera_state;
	uint8_t target_state;
	uint8_t hatch_state;
	uint8_t update;
	uint8_t move_state;
	uint8_t mode;
	uint8_t level;
	uint8_t state;
	uint8_t danger;
}game_state_t;


typedef __packed struct {
	uint8_t NUC_color1;
	uint8_t NUC_color2;
	uint8_t NUC_color3;
	uint8_t hatch_color;
	uint8_t capmode_color;
	uint8_t charge_color;
	uint8_t cap_rectangle_color;
	uint8_t capvolt_color;
	uint8_t shoot_color;
	uint8_t rotate_color;
	uint8_t power_color;
	uint8_t heat_color;
}color_test_t;

extern game_state_t each_state;
extern color_test_t color_test;
extern UI_t UI;

/**
  * @brief  ��0��ͼ�㻭�ַ��ĳ����30���ַ�
  * @param[in] robot_id  	  �û����˵ķ���ID
	* @param[in] string  		  �ַ��������������׵�ַ
	* @param[in] string_dex   ���ַ��ı�ʶ��������ѡ�и��ַ�����ɾ�����޸ĵĲ���
	* @param[in] control_way  ����������0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
	* @param[in] color        ��ɫ��0��������ɫ��1����ɫ��2����ɫ��4���Ϻ�ɫ�ȵ�
	* @param[in] layer        ���ַ������Ƶ�ͼ�㣬��0-9 ʮ��ͼ�㣬ÿ��ͼ����Ի����߸�ͼ��
	* @param[in] x        		����λ������ 0-1023 ��11bit��
	* @param[in] y        		����λ������ 0-1023 ��11bit��
	*/
void referee_draw_string(uint8_t robot_id,char *string,uint8_t string_dex,uint8_t control_way,uint8_t color,uint8_t on_layer, uint16_t x,uint16_t y);


/**
  * @brief  ��1��ͼ�㻭�������ĳ��򣬱�����λС��
  * @param[in] robot_id  	  �û����˵ķ���ID
	* @param[in] float_data  	������
	* @param[in] float_index  �ø������ı�ʶ��������ѡ�иø���������ɾ�����޸ĵĲ���
	* @param[in] control_way  ����������0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
	* @param[in] color        ��ɫ��0��������ɫ��1����ɫ��2����ɫ��4���Ϻ�ɫ�ȵ�
	* @param[in] x        		����λ������ 0-1023 ��11bit��
	* @param[in] y        		����λ������ 0-1023 ��11bit��
	*/
void referee_draw_float(uint8_t robot_id, float float_data,uint8_t float_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y);
	
void referee_draw_int(uint8_t robot_id,int32_t int_input,uint8_t int_index, uint8_t control_way,uint8_t color,uint16_t x,uint16_t y);

void referee_draw_line(uint8_t robot_id, uint8_t line_index, uint8_t control_way,uint8_t color,uint16_t x,uint16_t y,uint16_t end_x,uint16_t end_y);

void referee_draw_circle(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y, uint16_t R);
	
void referee_draw_rectangle(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y, uint16_t end_x, uint16_t end_y);

void referee_draw_ellipe(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y);

void referee_draw_arc(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y);
/*����ͬ�ϻ���ͬͼ��*/


/*Э��������ݵķ���*/
void draw_graph(graphic_data_struct_t *patterning, uint16_t index, uint8_t control_way, uint8_t graph, uint8_t layer, uint8_t color, uint8_t Sa, uint8_t Ea, uint8_t With,  uint16_t x,uint16_t y, uint8_t R, uint16_t Ex, uint16_t Ey);

/*ɾ��ͼ�����*/
void referee_graphic_delete(uint8_t del_layer, uint8_t operation, uint8_t robot_id);

/*��ͼ��Ҫ����*/
void referee_draw_nuc(uint8_t robot_id, uint8_t control_line2, uint8_t control_rectangle, uint8_t control_line, uint8_t control_hatch);
void referee_draw_cap(uint8_t robot_id, uint8_t control_line_cap, uint8_t control_line_charge, uint8_t control_cap_lenghth, float VOLT);
void referee_draw_shoot(uint8_t robot_id);
void referee_draw_robot(uint8_t robot_id, uint8_t level, uint8_t state, uint8_t danger, uint8_t control_level, uint8_t control_state, uint8_t control_danger);
void referee_draw_circle_shoot(uint8_t robot_id,uint8_t circle_index, uint8_t control_way, uint16_t x,uint16_t y);
	
/*���õĺ���*/
void referee_draw_supercap_data(uint32_t cnt, uint8_t robot_id, uint8_t cap_state, uint8_t charge_state, float capvolt);
void referee_draw_NUC_data(uint32_t cnt , uint8_t robot_id, uint8_t camera_state, uint8_t target_state, uint16_t mode, uint8_t hatch_state);
void referee_draw_Load_data(uint32_t cnt , uint8_t robot_id, uint8_t hatch_state);
void referee_draw_chassis_data(uint32_t cnt , uint8_t robot_id, uint8_t move_state, uint8_t power_state, uint8_t heat_state, float pitch_angle);
void referee_draw_shoot_data(uint32_t cnt , uint8_t robot_id, uint8_t mode, uint16_t x,uint16_t y);
void draw_robotdata(uint32_t cnt , uint8_t robot_id, uint8_t level, uint8_t state, uint8_t danger, float float_data);

/*����ͼ�Σ�û�����С��ͼ��Ҳû���*/
void update(uint32_t cnt, uint8_t robot_id, uint8_t change);
void draw_client_map_information(uint8_t robot_id, float x, float y);
void draw_interaction(uint8_t robot_id, uint16_t data);

/**
  * @brief  ���ݸû����˵ķ���ID�Զ��ж϶�Ӧ�Ŀͻ���ID
  * @param[in]  robot_id  	  �û����˵�ID
  * @param[out] client_id  	  Ŀ��ͻ��˵�ID
	*/
uint16_t referee_get_receiver_ID(uint32_t sender_id);


/*
0		3		ͼ����
3		4		b0-b2		ͼ�β���
								0 ��
								1 ����
								2	�޸�
								3	ɾ��
				b3-b5		ͼ������
								0 ֱ��
								1 ����
								2 ��Բ
								3 ��Բ
								4	Բ��
								5 ������
								6 ������
								7 �ַ�
				b6-b9		ͼ���� 0-9
				b10-b13	��ɫ
								0 ������ɫ 1�� 2�� 3�� 4�Ϻ� 5�� 6�� 7�� 8��
				b14-b22	��ʼ�Ƕ� 0-360
				b32-b31 ��ֹ�Ƕ� 0-360
7		4		b0-b9		�߿�
				b10-b20	���x
				b21-b31	���y
11	4		b0-b9		�����С��뾶
				b10-b20	�յ�x
				b21-b31	�յ�y
*/



extern uint8_t Data_Pack[128]__attribute__((at(0x24065000)));

unsigned char Get_CRC8_Check(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

void Interact_Pack_Shoot(void);
void Client_Display(void);

#endif
