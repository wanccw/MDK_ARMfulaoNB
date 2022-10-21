#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_
#include "stm32h7xx_hal.h"

#include "driver.h"
#include "Gyro.h"
#include "motor.h"
#include "mecanum_chassis.h"
#include "swerve_chassis.h"


#include "pid.h"

typedef struct
{
	float Vision_FPS;
	float Receiver_FPS;
	float Referee_FPS;
	float Holder_FPS;
	int Receiver_cnt;
	int Referee_cnt;
	int Vision_cnt;
	int Holder_cnt;
}FPS;

typedef struct
{
	float MpuAngle1;
	float CanAngle1;
	float MpuAngle2;
	float CanAngle2;
	float MpuAngle3;
  float CanAngle3;
	float MpuAngleChange;
	float CanAngleChange;
	float SenceBili;
}GetAngle;

typedef struct
{
	GetAngle Yaw;
	GetAngle Pitch;	
	uint16_t mpuDebugTime;
	uint8_t  DebugBeginFlag;
	uint8_t  DebugFinishFlag;
}MpuDebug;
	

uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);

uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);

uint8_t Mpu6050_senceDebug(MpuDebug* mpuDebug,gyro_data_t* gyro_data);

void TIM14_Task(void);

extern FPS tim14_FPS;
extern  uint16_t Reset_Count;
extern int asd;
extern float sens_pitch_change;
extern int time_cut;
extern int16_t ThisSecond;
extern MpuDebug mpudebug;
#endif



