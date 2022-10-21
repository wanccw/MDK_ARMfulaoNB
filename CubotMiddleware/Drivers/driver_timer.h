#ifndef DRIVERTIMER_H
#define DRIVERTIMER_H
#include "tim.h"


typedef void (*TIM_ElapsedCallback)(void); 


typedef struct
{
	TIM_HandleTypeDef* 	Handle;
//	uint32_t ClockTime;             			//< ����ʱ���ļ�������
	int32_t ClockTime;
	TIM_ElapsedCallback		ElapCallback;		//< ��ʱ������ж�
	uint16_t HolderTime;
	uint16_t ErrorTime;
	uint16_t UI_Time;
}TIM_Object;

/**
  * @brief ��ʼ����ʱ�����û��ص�
  */
void TIMx_Init(TIM_HandleTypeDef* handle, TIM_ElapsedCallback callback);


/**
  * @brief     ������ʱ������ж�
  */
void TIM_Open(TIM_Object* tim);



extern TIM_Object tim14;;


#endif

