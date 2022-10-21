#ifndef BRAIN_H__
#define BRAIN_H__
#include "stm32h7xx.h"
#include "usart.h"
#include "holder.h"
#include "dr16.h"
#include "vision.h"
#define Brain_rxBufferLengh 15
#define Velocity10 0x0A     //10��ÿ��
#define	Velocity11 0x0B
#define	Velocity12 0x0C
#define	Velocity13 0x0D
#define	Velocity14 0x0E
#define	Velocity15 0x0F
#define	Velocity16 0x10
#define	Velocity17 0x11
#define	Velocity18 0x12
#define	Velocity19 0x13
#define	Velocity20 0x14   

/**
  * @brief    �ں˵Ļص������������ڽ������ݺ�ʵʱִ��ָ��
  * @param[in]  
  */
typedef uint8_t (*Brain_CoreCallback)(Holder_t* holder, RC_Ctrl* rc_ctrl); 


/**
  * @brief  ��λ������֡����
  */
typedef enum
{
	BRAIN_TO_ROBOT_CMD   = 1,	 //< 0b0001
	BRAIN_TO_ROBOT_HINT  = 2,	 //< 0b0010
	ROBOT_TO_BRAIN_QUEST = 3,  //< 0b0011
	ROBOT_TO_BRAIN_LOG   = 4,	 //< 0b0100
	ROBOT_TO_BRAIN_CMD   = 5
}BrainFrameType;


/**
  * @brief  ��λ��������λ������
  */
typedef enum
{
	REBOOT_ALL_CORE   = 1,	 //< 0b0001
	REBOOT_BRAIN  = 2,	 //< 0b0010
	REBOOT_NUC = 3,  //< 0b0011
}RobotToBrainCmdCode;



/**
  * @brief  �ں�ģʽ����
  */
typedef enum
{
	MANUAL   = 0,               //�ֶ������ģʽ
	SHOOTONE = 1,               //���ȹ���1�ų�
	SHOOTTWO = 2,               //���ȹ���2�ų�
	SHOOTTHREE = 3,             //���ȹ���3�ų�
	SHOOTFOUR = 4,              //���ȹ���4�ų�
	SHOOTFIVE = 5,              //���ȹ���5�ų�
	SHOOTSENTRY = 6,            //���ȹ����ڱ�
	SHOOTOUTPOST = 7,           //���ȹ���ǰ��վ
	SHOOTBASE = 8,              //���ȹ������
	AUTOMATICHIT = 9,           //�Զ����ģʽ
	CURVEDFIREOUTPOST = 10,     //����ǰ��վ
	CURVEDFIREBASE = 11,        //�������
	SHOOTSMALLBUFF = 12,        //��С��
	SHOOTLARGEBUFF = 13,        //����
}BrainModeHint;   

/**
  * @brief  ��λ�������ں˽ṹ��
  */
typedef struct
{
	uint8_t CoreID;					//< �ں˹̶�ID
	uint8_t BrainMode;  		//< ��������˴����ں��л��Ĺ���ģʽ��
	uint8_t BrainVelocity;  //< ��������˴����ں��л����ӵ����ٶȡ�
	
	struct 
	{
		float YawDeflectionAngle;
		float PitchDeflectionAngle;
		float Distance;
		uint8_t IsFire;      //������
	}CoreInstruction; 

	struct 
	{
		uint8_t Working;     //�����˴����ں˵��������״̬ 
		uint8_t Connect;     //�����˴����ں˵��������״̬
		uint8_t Open;        //�����˴����ں˵Ĵ�״̬
 		uint8_t Init;        //�����˴����ں˵ĳ�ʼ��״̬
	}CoreFlag;  //0�� 1��

	Brain_CoreCallback CoreCallback;	
	
}BrainCore_t;         

typedef struct 
	{
 uint8_t forecast_cut;
 uint8_t forecast_flag;
 float yaw_sum;
 float pitch_sum;
 float yaw_average;
 float pitch_average;
 float yaw_error;
 float pitch_error;
 float yaw_add;
 float pitch_add;
	}Forecast;  //�ƶ�Ԥ�ⲹ��

/**
  * @brief  ��λ�����Խṹ��
  */
typedef struct
{ 
	uint8_t FrameType;
	uint8_t FrameCoreID;
	BrainCore_t BrainCore[4];
}CubotBrain_t;           


/**
  * @brief      ��λ�����ݲ�ֽ��㺯��
	* @param[in]  brain      ��λ�����ݽṹ��
	* @param[in]  recBuffer  ��������������
  */
void Brain_DataUnpack(CubotBrain_t* brain,Trace* Info_Vision, uint8_t* recBuffer);

/**
* @brief      ��λ������λ����������ĺ���
	* @param[in]  mode       ��λ������ģʽ
	* @param[in]  coreID     ��λ���ں˱�ǩ
	* @param[in]  velocity   ǹ�ڳ��ٶ�
  */
void Brain_RobotToBrainQuest(uint8_t mode, uint8_t coreID, uint8_t velocity);
void Brain_RobotToComputerCmd(uint8_t Type);
void Brain_RobotToBrainQuest_WorkingModel(uint8_t mode, uint8_t coreID,uint8_t Type);
void Brain_RobotToBrainQuest_XinTiaoBao(uint8_t Type);
void Brain_RobotToBrainQuest_Velocity(uint8_t coreID,uint8_t Type,uint8_t Velocity);
void Brain_RobotToBrainLog(char* log_string, uint8_t coreID,uint8_t Type);


/**
	* @brief      ��λ������λ�����Ϳ�����Ϣ
	* @param[in]  command    ��λ��������λ��������
	* @param[in]  coreID     ��λ���ں˱�ǩ
  */
void Brain_RobotToBrainCmd(uint8_t Type);
uint8_t Brain_callback(uint8_t * recBuffer, uint16_t len);
void Brain_RobotToBrainTime(float yaw);

void Brain_RobotToBrainLog(char* log_string, uint8_t coreID,uint8_t Type);
void Brain_RobotModeControl(Trace* Info_Vision,RC_Ctrl* rc_ctrl);

extern UART_RxBuffer uart2_buffer;
extern CubotBrain_t Brain;
extern float YawSence;
extern float PitchSence;
extern float LastYawDeflectionAngle;
extern float LastPitchDeflectionAngle;
extern int PitchDeflectionAngleCut;
extern float Brain_yaw_add;
extern float Brain_pitch_add;
extern float Brain_add_cut;
extern uint8_t ShootModeToBrain; 
extern uint16_t dafu_cut;
extern uint8_t dafu_shoot_flag;
extern float  BrainTenYawAngle[10];
extern float  BrainTenPitchAngle[10];;
#endif

