/**@file    motor.c
* @brief    �豸�㣬������ƴ��롣�豸������������������ݽṹ�������豸����ɹ���������¶�o�û�
* @details  ͨ������CAN��������մ���ͷ��͵�����������
* @date        2021-10-10
* @version     V1.0
* @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
**********************************************************************************
* @attention
* Ӳ��ƽ̨: STM32H750VBT \n
* SDK�汾��-++++
* @par �޸���־:
* <table>
* <tr><th>Date       <th>Version  <th>Author    <th>Description
* <tr><td>2021-10-10  <td>1.0      <td>RyanJiao  <td>����շ�������д
* </table>
*
**********************************************************************************
 ==============================================================================
                          How to use this driver  
 ==============================================================================

	���driver_can.h
	
	1. ����Motor�ṹ�壬��Ϊ���ʵ����
	
	1. ����MotorInit()����ʼ�������̬���ݡ�

	2. ������CANʱע���CANx_rxCallBack�ص����ж�ID�����MotorRxCallback()�����յ����̬���ݡ�
	
	3. ����PID������������������OutputCurrent��
	
	4. ��������Ӧ������MotorFillData()��д��Ӧ����ID�µĴ���������
	
	5. ����������д��Ϻ����MotorCanOutput()���Ͷ�ӦCAN�豸���ض�����ID��CAN����

  ********************************************************************************
	* @attention
	* Ӳ��ƽ̨: STM32H750VBT \n
	* SDK�汾��-++++
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.		
***********************************************************************************/
#include "motor.h"
int Q_index = 0;
float Ecd_sum = 0;
float Speed_sum = 0;
/**
  * @brief ��Ӧ�󽮵����ͬ����ID��CAN���ݷ��ͻ�����
  */
CAN_TxBuffer txBuffer0x200forCAN1={
	.Identifier = 0x200
};
CAN_TxBuffer txBuffer0x1FFforCAN1={
	.Identifier = 0x1ff
};
CAN_TxBuffer txBuffer0x2FFforCAN1={
	.Identifier = 0x2ff
};
CAN_TxBuffer txBuffer0x200forCAN2={
	.Identifier = 0x200
};
CAN_TxBuffer txBuffer0x1FFforCAN2={
	.Identifier = 0x1ff
};
CAN_TxBuffer txBuffer0x2FFforCAN2={
	.Identifier = 0x2ff
};



/**
  * @brief  ���������㺯�����������̶�ת��Ϊ�Ƕ�
  */
static void MotorEcdtoAngle(Motor* motor)
{
	if((&motor->Param)->EcdOffset < ((&motor->Param)->EcdFullRange/2))
	{
		if((motor->Data.Ecd) > (&motor->Param)->EcdOffset + (&motor->Param)->EcdFullRange/2)
			(&motor->Data)->Ecd = (motor->Data.Ecd) - (&motor->Param)->EcdFullRange;
	}
	else
	{
		if((motor->Data.Ecd) < (&motor->Param)->EcdOffset - (&motor->Param)->EcdFullRange/2)
			(&motor->Data)->Ecd = (motor->Data.Ecd) + (&motor->Param)->EcdFullRange;
	}
		(&motor->Data)->Angle = K_ECD_TO_ANGLE * ((motor->Data.Ecd) - (&motor->Param)->EcdOffset);
}
static void MotorLvboEcdtoAngle(Motor* motor){
if((&motor->Param)->EcdOffset < ((&motor->Param)->EcdFullRange/2))
	{
		if((motor->Data.LvboEcd) > (&motor->Param)->EcdOffset + (&motor->Param)->EcdFullRange/2)
			(&motor->Data)->LvboEcd = (motor->Data.LvboEcd) - (&motor->Param)->EcdFullRange;
	}
	else
	{
		if((motor->Data.LvboEcd) < (&motor->Param)->EcdOffset - (&motor->Param)->EcdFullRange/2)
			(&motor->Data)->LvboEcd = (motor->Data.LvboEcd) + (&motor->Param)->EcdFullRange;
	}
		(&motor->Data)->LvboAngle = K_ECD_TO_ANGLE * ((motor->Data.LvboEcd) - (&motor->Param)->EcdOffset);
}


/**
  * @brief  �������޷���
  */
static void MotorOutputLimit(Motor* motor)
{
	if((&motor->Data)->Output > motor->Param.CurrentLimit)
		(&motor->Data)->Output = motor->Param.CurrentLimit;
	else if((&motor->Data)->Output < (-motor->Param.CurrentLimit))
		(&motor->Data)->Output = (-motor->Param.CurrentLimit);
}



/**
  * @brief  ���C610��C620����Ŀ���ID������������������CAN���ͻ������ĺ���ָ�롣
  */
static  uint8_t CAN_fill_3508_2006_data(CAN_Object can, MotorData motor_data, uint16_t id)
{
	if(can.Handle == &hfdcan1)
	{
		if(id >= 0x201 && id <= 0x204)
		{
		 txBuffer0x200forCAN1.Data[(id - 0x201) * 2] = motor_data.Output >> 8;
		 txBuffer0x200forCAN1.Data[(id - 0x201) * 2 + 1] = motor_data.Output & 0xff;		
		}
		else if(id >= 0x205 && id<= 0x208)
		{
		 txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2] = motor_data.Output >> 8;
		 txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;	
		}
	}
	else if(can.Handle == &hfdcan2)
	{
		if(id >= 0x201 && id <= 0x204)
		{
		 txBuffer0x200forCAN2.Data[(id - 0x201) * 2] = motor_data.Output >> 8;
		 txBuffer0x200forCAN2.Data[(id - 0x201) * 2 + 1] = motor_data.Output & 0xff;		
		}
		else if(id >= 0x205 && id<= 0x208)
		{
		 txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2] = motor_data.Output >> 8;
		 txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;	
		}
		else if(id >= 0x209 && id<= 0x20B)
		{
		 txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2] = motor_data.Output >> 8;
		 txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2 + 1] = motor_data.Output & 0xff;	
		}
	}
	return 0;
}


/**
  * @brief  ���GM6020����Ŀ���ID������������������CAN���ͻ������ĺ���ָ�롣
  */
static  uint8_t CAN_fill_6020_data( CAN_Object can, MotorData motor_data,uint16_t id)
{
	if(can.Handle == &hfdcan1)
	{
		if(id >= 0x205 && id <= 0x208)
		{
		 txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2] = motor_data.Output >> 8;
		 txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;		
		}
		else if(id >= 0x209 && id<= 0x20B)
		{
		 txBuffer0x2FFforCAN1.Data[(id - 0x209) * 2] = motor_data.Output >> 8;
		 txBuffer0x2FFforCAN1.Data[(id - 0x209) * 2 + 1] = motor_data.Output & 0xff;	
		}
	}
	else if(can.Handle == &hfdcan2)
	{
		if(id >= 0x205 && id <= 0x208)
		{
		 txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2] = motor_data.Output >> 8;
		 txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;		
		}
		else if(id >= 0x209 && id<= 0x20B)
		{
		 txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2] = motor_data.Output >> 8;
		 txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2 + 1] = motor_data.Output & 0xff;	
		}
	}
	return 0;
}


/**
  * @brief  ������ݸ��»ص�������ֻ��motor.c�ļ��ڵ��á����󽮵���������ĸ�ʽ��ͬ��
  */
static  uint8_t CAN_update_data(MotorData* motor, CAN_RxBuffer rxBuffer)
{
	motor->LastEcd       = motor->Ecd;      //< ���±������Ƕ�ǰ��¼�ϸ����ڵı������Ƕ�
	motor->RawEcd 			 = rxBuffer.Data[0]<<8|rxBuffer.Data[1];
	motor->SpeedRPM      = rxBuffer.Data[2]<<8|rxBuffer.Data[3];
	motor->TorqueCurrent = rxBuffer.Data[4]<<8|rxBuffer.Data[5];
	motor->Temperature   = rxBuffer.Data[6];
	motor->Ecd           = motor->RawEcd;

	return 0;
}


/**
  * @brief ע�����豸��CAN�豸������
  */
static void CAN_RegisteMotor(CAN_Object* canx, Motor* motor)
{
	list_add(&motor->list, (&canx->DevicesList));
}


/**
  * @brief ������ṹ���CAN�豸����ɾ��
  */
static void CAN_DeleteMotor(Motor* motor)
{
  list_del(&(motor->list)); //< �ж�������������ɾ�����豸
}



/**
  * @brief  �����ʼ�������þ�̬������������������λ��������ͺ�id��
  */
void MotorInit(Motor* motor, uint16_t ecd_Offset, MotorType type, CanNumber canx, uint16_t id)
{
	(&motor->Param)->EcdOffset = ecd_Offset;
	(&motor->Param)->MotorType = type;
	(&motor->Param)->CanId     = id;
	(&motor->Param)->CanNumber = canx;
	
	if(canx == CAN1)
		CAN_RegisteMotor(&can1, motor);
	else if(canx == CAN2)
		CAN_RegisteMotor(&can2, motor);
	
	switch (type)
	{
		case Motor3508: 
		{
			(&motor->Param)->CurrentLimit = CURRENT_LIMIT_FOR_3508;
			(&motor->Param)->EcdFullRange = ECD_RANGE_FOR_3508;
			motor->MotorUpdate            = CAN_update_data;
			motor->FillMotorData          = CAN_fill_3508_2006_data;
			break;
		}
		case Motor6020: 
		{
			(&motor->Param)->CurrentLimit = CURRENT_LIMIT_FOR_6020;
			(&motor->Param)->EcdFullRange = ECD_RANGE_FOR_6020;
			motor->MotorUpdate            = CAN_update_data;
			motor->FillMotorData          = CAN_fill_6020_data;
			break;
		}
		case Motor2006: 
		{
			(&motor->Param)->CurrentLimit = CURRENT_LIMIT_FOR_2006;
			(&motor->Param)->EcdFullRange = ECD_RANGE_FOR_2006;
			motor->MotorUpdate            = CAN_update_data;
			motor->FillMotorData          = CAN_fill_3508_2006_data;
			break;
		}
		default: ;
	}
}


/**
  * @brief  ����canID���豸������Ѱ�Ҷ�Ӧ�ĵ��
  */
static Motor* MotorFind(uint16_t canid, CAN_Object canx)
{
	Motor* motor = NULL;
	list_t *node = NULL;	
	
	for (node = canx.DevicesList.next;    		//< ��ѭ���������һȦ
			 node != (canx.DevicesList.prev->next);
			 node = node->next)
	{
		motor = list_entry(node, Motor, list);  //< ��������ͷ�����ڽ�㡢��Ƕ������Ľṹ�����͡���Ƕ������Ľṹ������������������ƣ����ɷ���Ƕ��ͷ�����ڽ��Ľṹ��
		if (motor->Param.CanId == canid)
		{
			return motor;
		}

	}
		return NULL;
}



/**
  * @brief  ������ջص�ҵ���߼�, ���µ����̬���ݣ����б������Ƕȱ任
  */
void MotorRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer)
{
	uint32_t id;	
	Motor* temp_motor = NULL;
	id = rxBuffer.Header.Identifier;
	temp_motor = MotorFind(id,canx);
	if(temp_motor != NULL)
	{
		temp_motor->MotorUpdate(&temp_motor->Data, rxBuffer); 
		temp_motor->Data.CanEcd[Q_index] = temp_motor->Data.Ecd;
		temp_motor->Data.CanAngleSpeed[Q_index] = temp_motor->Data.SpeedRPM;
		Q_index++;
		if(Q_index==20)Q_index=0;
		for(int i = 0; i <20 ;i++){
    Ecd_sum += temp_motor->Data.CanEcd[Q_index];
		Speed_sum += temp_motor->Data.CanAngleSpeed[Q_index];			
		}
		temp_motor->Data.LvboSpeedRPM = Speed_sum/20;
		temp_motor->Data.LvboEcd = Ecd_sum/20;
		Speed_sum = 0;
		Ecd_sum = 0;
		MotorEcdtoAngle(temp_motor);
		MotorLvboEcdtoAngle(temp_motor);
	}
}


/**
  * @brief ��õ���ṹ���е�ID
  */
uint16_t MotorReturnID(Motor motor)
{
	return motor.Param.CanId;
}



/**
  * @brief  ��motor_data.Output�޷������뷢�ͻ������ȴ����͡�
	*/
int16_t amp_test;
void MotorFillData(Motor* motor, int32_t output)
{
	motor->Data.Output = output;
	MotorOutputLimit(motor);
	amp_test = motor->Data.Output;
	
	if(motor->Param.CanNumber == CAN1)
		motor->FillMotorData(can1, motor->Data, motor->Param.CanId);
  else if(motor->Param.CanNumber == CAN2)
		motor->FillMotorData(can2, motor->Data, motor->Param.CanId);
}



/**
  * @brief  ���ض�ID��CAN_TxBuffer���ͳ�ȥ��
  */
uint16_t MotorCanOutput(CAN_Object can, int16_t IDforTxBuffer)
{
	switch (IDforTxBuffer)
	{
		case 0x200: 
		{
			if(can.Handle == &hfdcan1)
				CAN_Send(&can, &txBuffer0x200forCAN1);
			else if(can.Handle == &hfdcan2)
				CAN_Send(&can, &txBuffer0x200forCAN2);
			break;
		}
		case 0x1ff: 
		{
			if(can.Handle == &hfdcan1)
				CAN_Send(&can, &txBuffer0x1FFforCAN1);
			else if(can.Handle == &hfdcan2)
				CAN_Send(&can, &txBuffer0x1FFforCAN2);
			break;
		}
		case 0x2ff: 
		{
			if(can.Handle == &hfdcan1)
				CAN_Send(&can, &txBuffer0x2FFforCAN1);
			else if(can.Handle == &hfdcan2)
				CAN_Send(&can, &txBuffer0x2FFforCAN2);
			break;
		}
		default: ;
	}
	return 0;
}

