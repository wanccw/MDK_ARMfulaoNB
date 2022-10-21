#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32h7xx_hal.h"
#include "driver_can.h"
#include "linux_list.h" 


#define K_ECD_TO_ANGLE 0.043945f  		//< �Ƕ�ת���������̶ȵ�ϵ����360/8192
#define ECD_RANGE_FOR_3508 8191				//< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_3508 16000   //< ���Ƶ�����ΧΪ����16384
#define ECD_RANGE_FOR_6020 8191				//< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_6020 29000   //< ���Ƶ�����ΧΪ����30000
#define ECD_RANGE_FOR_2006 8191				//< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_2006 10000   //< ���Ƶ�����ΧΪ����16384


/**
  * @brief  ���������࣬���ڷ��ͺ�����ѡ��
	* @note   GM6020�ķ�������IDΪ 0x205-0x20B ֮��
	* @note   M3508�ķ�������IDΪ 0x201-0x208 ֮��
  */
typedef enum 
{
	Motor3508 = 0x00U,
	Motor6020 = 0x01U,
	Motor2006 = 0x02U
}MotorType;


/**
  * @brief  �����̬���ݣ���������в��������ݣ���CAN�жϻص�����
  */
typedef struct
{
	int16_t  Ecd;         	//< ��ǰ����������ֵ
	int16_t  SpeedRPM;			//< ÿ������תȦ��
	int16_t  TorqueCurrent; //< ��������
	uint8_t  Temperature;	  //< �¶�
	
	int16_t  RawEcd;				//< ԭʼ����������
	int16_t  LastEcd;			  //< ��һʱ�̱���������ֵ			
	float    Angle;					//< �����ı������Ƕ�
	int16_t  AngleSpeed;	  //< �����ı��������ٶ�	
	int32_t  RoundCnt;			//< �ۼ�ת��Ȧ��
	int32_t  TotalEcd;			//< �������ۼ�����ֵ
	int32_t  TotalAngle;		//< �ۼ���ת�Ƕ�

	int16_t  Target;				//< �������������
	int32_t  Output;  			//< ������ֵ��ͨ��Ϊ�����͵�ѹ	
	float CanEcd[20] ;
	float CanAngleSpeed[20] ;
	float LvboAngle;
	int16_t  LvboEcd;
	int16_t  LvboSpeedRPM;
	
}MotorData;


/**
  * @brief   ����������ڳ�ʼ��������ȷ��
  */
typedef struct 														
{
	uint8_t  CanNumber;			 										//< �����ʹ�õ�CAN�˿ں�
	uint16_t CanId;			 												//< ���ID	
	uint8_t  MotorType;			 										//< �������	
	uint16_t EcdOffset;	 									  		//< �����ʼ���
	uint16_t EcdFullRange;											//< ����������
	int16_t  CurrentLimit;			 								//< ����ܳ��ܵ�������
}MotorParam;



/**
  * @brief     ������Ĵ�������������CAN���ͻ�����
  * @param[in] motor_data     �����̬���ݽṹ�壬ֻ��Ҫ���ͣ����޸����ݣ�����Ҫ����ָ�롢
  * @param[in] id             �����ʶ��ID
  */
typedef uint8_t (*CAN_FillMotorData)(CAN_Object can, MotorData motor_data, uint16_t id); //< ������ͻص�


/**
  * @brief  ɸѡCAN���ݣ������µ����̬���ݵĻص�����
  */
typedef uint8_t (*Motor_DataUpdate)(MotorData* motor_data, CAN_RxBuffer rxBuffer); //< ������ͻص�



/**
  * @brief  ��������ݺͲ������Լ�������ͬ���֮����������ĳ�Ա����
  */
typedef struct 
{
	list_t             list;			 											//< ����ָ�룬����ָ���Լ��ĳ�ʼ������ͨ��ע�ắ����չ��ѭ������
	MotorData          Data;   	 											  //< �����̬���ݣ������и���
	MotorParam         Param;			 											//< ����������ڳ�ʼ��ʱ����
	
	Motor_DataUpdate   MotorUpdate;											//< ���µ���������ݵĺ���ָ��
	CAN_FillMotorData  FillMotorData;										//< �Բ�ͬ����ID��CAN���ͻ�����������������ݵĺ���ָ��
}Motor;



/**
  * @brief  		�����ʼ��
	* @note 			GM6020���IDΪ001ʱ����������Ϊ0x205,���IDΪ111ʱ��7������������IDΪ0x20B
  * @param[in]  motor      ������ݽṹ��
  * @param[in]  ecd_Offset  ��������ʼֵ
  * @param[in]  type       ������� (ö��)
  * @param[in]  canx       CAN�˿ں� (ö��)
  * @param[in]  id         ���ID��ʶ��
  */
void MotorInit(Motor* motor, uint16_t ecd_Offset, MotorType type, CanNumber canx, uint16_t id);


/**
  * @brief  		����Ľ��ջص�ҵ���߼�����CAN�����ж����ж�����ID�����
  * @param[in]  motor      ������ݽṹ��
  * @param[in]  rxBuffer   CAN�������ݻ�����
  */
void MotorRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer);



/**
  * @brief  		��õ���ṹ���е�ID
  * @param[in]  motor   ������ݽṹ��
  */
uint16_t MotorReturnID(Motor motor);



/**
  * @brief  		��motor_data.Output���뷢�ͻ������ȴ����͵ĺ�����
	* @note			  ����ķ�������ID�ڲ�ͬ�����ڣ����Ƶ�����ĵ�id��ͬ������Ҫ��д�˺�����
  * @param[in]  motor      ������ݽṹ��
  * @param[in]  output     ������Ŀ�����
*/
void MotorFillData(Motor* motor, int32_t output);




/**
  * @brief  		���ض�ID��CAN_TxBuffer���ͳ�ȥ
	* @note 			����ķ���ID���ã�Ӱ�����Ŀ���ID��	
	* @note 			�󽮵��3508���������Ϊ����ʱͨ��ʹ��0x200��Ϊ����ID���������ĸ����̵��ID����Ϊ0x201-0x204����0x205-0x208��ʹ����һ������ID��
  * @param[in]  can						  		CAN�豸�ṹ��
  * @param[in]  IDforTxBuffer  		  �������ID
  */
uint16_t MotorCanOutput(CAN_Object can, int16_t IDforTxBuffer);


extern int Q_index;
extern float Angle_sum;
extern float Speed_sum;
#endif // __MOTOR_H__
