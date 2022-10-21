#ifndef BRAIN_H__
#define BRAIN_H__
#include "stm32h7xx.h"
#include "usart.h"
#include "holder.h"
#include "dr16.h"
#include "vision.h"
#define Brain_rxBufferLengh 15
#define Velocity10 0x0A     //10米每秒
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
  * @brief    内核的回调函数，用于在结算数据后实时执行指令
  * @param[in]  
  */
typedef uint8_t (*Brain_CoreCallback)(Holder_t* holder, RC_Ctrl* rc_ctrl); 


/**
  * @brief  上位机数据帧汇总
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
  * @brief  下位机控制上位机汇总
  */
typedef enum
{
	REBOOT_ALL_CORE   = 1,	 //< 0b0001
	REBOOT_BRAIN  = 2,	 //< 0b0010
	REBOOT_NUC = 3,  //< 0b0011
}RobotToBrainCmdCode;



/**
  * @brief  内核模式汇总
  */
typedef enum
{
	MANUAL   = 0,               //手动把射击模式
	SHOOTONE = 1,               //优先攻打1号车
	SHOOTTWO = 2,               //优先攻打2号车
	SHOOTTHREE = 3,             //优先攻打3号车
	SHOOTFOUR = 4,              //优先攻打4号车
	SHOOTFIVE = 5,              //优先攻打5号车
	SHOOTSENTRY = 6,            //优先攻打哨兵
	SHOOTOUTPOST = 7,           //优先攻打前哨站
	SHOOTBASE = 8,              //优先攻打基地
	AUTOMATICHIT = 9,           //自动射击模式
	CURVEDFIREOUTPOST = 10,     //吊射前哨站
	CURVEDFIREBASE = 11,        //吊射基地
	SHOOTSMALLBUFF = 12,        //打小符
	SHOOTLARGEBUFF = 13,        //打大符
}BrainModeHint;   

/**
  * @brief  上位机大脑内核结构体
  */
typedef struct
{
	uint8_t CoreID;					//< 内核固定ID
	uint8_t BrainMode;  		//< 请求机器人大脑内核切换的工作模式。
	uint8_t BrainVelocity;  //< 请求机器人大脑内核切换的子弹初速度。
	
	struct 
	{
		float YawDeflectionAngle;
		float PitchDeflectionAngle;
		float Distance;
		uint8_t IsFire;      //开火建议
	}CoreInstruction; 

	struct 
	{
		uint8_t Working;     //机器人大脑内核的相机工作状态 
		uint8_t Connect;     //机器人大脑内核的相机连接状态
		uint8_t Open;        //机器人大脑内核的打开状态
 		uint8_t Init;        //机器人大脑内核的初始化状态
	}CoreFlag;  //0关 1开

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
	}Forecast;  //移动预测补偿

/**
  * @brief  上位机大脑结构体
  */
typedef struct
{ 
	uint8_t FrameType;
	uint8_t FrameCoreID;
	BrainCore_t BrainCore[4];
}CubotBrain_t;           


/**
  * @brief      上位机数据拆分解算函数
	* @param[in]  brain      上位机数据结构体
	* @param[in]  recBuffer  缓存区数组数据
  */
void Brain_DataUnpack(CubotBrain_t* brain,Trace* Info_Vision, uint8_t* recBuffer);

/**
* @brief      下位机向上位机发送请求的函数
	* @param[in]  mode       上位机工作模式
	* @param[in]  coreID     上位机内核标签
	* @param[in]  velocity   枪口初速度
  */
void Brain_RobotToBrainQuest(uint8_t mode, uint8_t coreID, uint8_t velocity);
void Brain_RobotToComputerCmd(uint8_t Type);
void Brain_RobotToBrainQuest_WorkingModel(uint8_t mode, uint8_t coreID,uint8_t Type);
void Brain_RobotToBrainQuest_XinTiaoBao(uint8_t Type);
void Brain_RobotToBrainQuest_Velocity(uint8_t coreID,uint8_t Type,uint8_t Velocity);
void Brain_RobotToBrainLog(char* log_string, uint8_t coreID,uint8_t Type);


/**
	* @brief      下位机向上位机发送控制信息
	* @param[in]  command    下位机控制上位机命令码
	* @param[in]  coreID     上位机内核标签
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

