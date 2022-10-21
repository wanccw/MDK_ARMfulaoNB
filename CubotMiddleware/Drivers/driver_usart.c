/**@file  driver_usart.c
* @brief   �����㣬���ڹ����������ļ����û��ص��ض���
* @details  ��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
* @author      RyanJiao  any question please send mail to 1095981200@qq.com						 
* @date        2021-8-23
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
**********************************************************************************
* @attention
* Ӳ��ƽ̨: STM32H750VBT \n
* SDK�汾��-++++
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>������ʼ�汾
* </table>
*
**********************************************************************************
 ==============================================================================
                          How to use this driver  
 ==============================================================================
 
	���driver_can.h

	1. ��UARTx_Init() �� ��� �� �û�����Ľ��ջص����� ������UART�ṹ��  ���ص������жԽ��յ������ݽ��� IDʶ�� �� �ϲ����㣩

  2. �û���д UART_RxBuffer������ Ŀ�껺������ַ �� ���ݳ��ȡ�

	3. ����UART_Open() ���� UART_Object �� �û���д UART_RxBuffer��
	
	4. �� UART_Idle_Handler ��ӵ� stm32H7xx_it.c �� USARTx_IRQHandler() �У������û���д��ͬһ�� UART_RxBuffer ��
	
	5. Ӧ�ò��д UART_TxBuffer �����ͻ������ṹ�壩������������ֽ������׵�ַ���ֽڳ���
	
	6. ��UART_Send()���� UART�豸�ṹ�� �� UART_TxBuffer�ṹ�壬�����ݷ��ͳ�ȥ

  ********************************************************************************
	* @attention
	* Ӳ��ƽ̨: STM32H750VBT \n
	* SDK�汾��-++++
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ********************************************************************************
	��DMA��NDTR�Ĵ���������
	This register can be written only
	when the stream is disabled. When the stream is enabled, this register is read-only,
	indicating the remaining data items to be transmitted. This register decrements after each
	DMA transfer.
	ָ����DMA�д������ʣ�����ݸ��� ÿ��DMA������ɺ��Զ���һ
	�ο��ֲ��ж�idle�����жϴ���������������
	
*/
#include "driver_usart.h"
#include "referee.h"
#include "usart.h"
#include "vision.h"
#include "hardware_config.h"
#include "stdio.h"
UART_Object uart1;
UART_Object uart2;
UART_Object uart3;
UART_Object uart4;
UART_Object uart5;
UART_Object uart6;


/**
  * @brief   ���ڳ�ʼ����������ͽ��ջص����������ڽṹ��
  */
void UARTx_Init(UART_HandleTypeDef* handle, UART_RxIdleCallback rxIdleCallback)
{
	// ��ʼ��uart1
	if (handle->Instance == USART1)
	{
		uart1.Handle = handle;
		uart1.RxIdleCallback = rxIdleCallback;
	}
	
	// ��ʼ��uart2
	if (handle->Instance == USART2)
	{
		uart2.Handle = handle;
		uart2.RxIdleCallback = rxIdleCallback;
	}
	
	// ��ʼ��uart3
	if (handle->Instance == USART3)
	{
		uart3.Handle = handle;
		uart3.RxIdleCallback = rxIdleCallback;
	}
	
	// ��ʼ��uart4
	if (handle->Instance == UART4)
	{
		uart4.Handle = handle;
		uart4.RxIdleCallback = rxIdleCallback;
	}
	
	// ��ʼ��uart5
	if (handle->Instance == UART5)
	{
		uart5.Handle = handle;
		uart5.RxIdleCallback = rxIdleCallback;
	}
	
	// ��ʼ��uart6
	if (handle->Instance == USART6)
	{
		uart6.Handle = handle;
		uart6.RxIdleCallback = rxIdleCallback;
	}
}


/**
  * @brief  ���ڹ������ṹ������Ѿ�Ԥ����д�õĴ����豸��ʼ��
  */
void UART_Receive_DMA(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
	HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
//	__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE);   		//<�����ж�ǰ����ص��ȽϺ�
} 


/**
  * @brief  ���ڹ������ṹ������Ѿ�Ԥ����д�õĴ����豸��ʼ��     //���ֲ����ֿ�ʹ�ã��Ӻ�DMA����
  */
void UART_ENABLE_IT(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
//	HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE);   		//<�����ж�ǰ����ص��ȽϺ�
} 

/**
  * @brief  ���ڵ���dma�������ݣ���������Ϊ�ֽ�
  */
uint32_t UART_Send(UART_Object* uart,UART_TxBuffer* txBuffer)
{
	return HAL_UART_Transmit_DMA(uart->Handle,txBuffer->Data, txBuffer->DataSize);
}



/**
  * @brief  �����豸�жϺ�����ִ���ж�DMA���������ô����û��ص����� 
  */
void UART_Idle_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
	assert_param(uart != NULL);
	
	uint16_t usart_rx_num;
	
	if((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET))
	{	
		HAL_UART_DMAStop(uart->Handle);																																					//< �ر�DMA����ֹ������������ݸ��£���ɶ�ʧ����															
		__HAL_UART_CLEAR_IDLEFLAG(uart->Handle);																																//< ���idle��־λ����ֹ�ٴν����ж�			
		__HAL_UART_CLEAR_OREFLAG(uart->Handle);	
//		if ((((DMA_Stream_TypeDef*)uartManage->huart.hdmarx->Instance)->NDTR) == uartManage->rxBufferSize)		//< �жϻ��������� �Ƿ���� DMA��δ�������ݳ��ȡ���Ϊ���ڣ����ݸ�����һ�Ρ� 
		usart_rx_num = rxBuffer->Size - ((DMA_Stream_TypeDef*)uart->Handle->hdmarx->Instance)->NDTR;  			//< 
		{																																																				//< ������ͷ�������������շ���ʼ���պ��Ƿ���Ҫƥ��ͷ�ֽں�β�ֽ�																																														
			if((*uart).RxIdleCallback!=NULL)
			  uart->RxIdleCallback(rxBuffer->Data, usart_rx_num);            				 		   												//<�û��ص�
		}
		HAL_UART_DMAResume(uart->Handle);		
		HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	}
}
//void USART2_IDLE_CALLBACK()
//{
//uint16_t temp,rx_len;
//	if(( __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!= RESET))//idle��־����λ
//	{ 	
//	//	HAL_UART_DMAStop(&huart2); 
//	  __HAL_DMA_DISABLE(&hdma_usart2_rx);
//		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//�����־λ
//		__HAL_UART_CLEAR_OREFLAG(&huart2);			 
//		
//		temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);// ��ȡDMA��δ��������ݸ���
//		rx_len = 6 - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
//	  Run_User_Vision_Task(&rc_Ctrl,&Holder,&Vision_Info);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,6);
////	__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx,DMA_FLAG_TCIF2_6|DMA_FLAG_HTIF2_6);
////		DMA1_Stream2->NDTR = (uint32_t)7;
//		__HAL_DMA_ENABLE(&hdma_usart2_rx);		
//		HAL_UART_Receive_DMA(&huart2,Vision_meta,6);//���´�DMA����  		
//	}
//}
//void USART3_IDLE_CALLBACK()
//{
//	uint8_t this_time_length;
//	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))
//	{  
//		HAL_UART_DMAStop(&huart3);
//		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
//		__HAL_UART_CLEAR_OREFLAG(&huart3);
//		this_time_length = BSP_USART3_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
//		Referee_Data_Diapcak(meta_data,this_time_length);
//		HAL_UART_Receive_DMA(&huart3,meta_data,BSP_USART3_DMA_RX_BUF_LEN);
//	}

//}

//uint8_t data_to_send_V6[20]__attribute__((at(0x24002000)));
//void ANO_V6_Send_Up_Computer(int16_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6)
//{
//	uint8_t _cnt=0;
//	int16_t _temp;
//	
//	uint8_t sum = 0;
//	uint8_t i=0;
//		/*   Э��ΪV6�汾 ע��۲�����   */
//	data_to_send_V6[_cnt++]=0xAA;
//	data_to_send_V6[_cnt++]=0x05; //����00Ҳ���� 05�������ؿ��߷ɿ�
//	data_to_send_V6[_cnt++]=0xAF; //AF����λ��ʶ����
//	data_to_send_V6[_cnt++]=0xF1; 
//	data_to_send_V6[_cnt++]=0;
//	
//	_temp = user1;    
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user2;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user3;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user4;    
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user5;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user6;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	
//	data_to_send_V6[6] = _cnt-5;
//	
//	sum = 0;
//	for(i=0;i<_cnt;i++)
//			sum += data_to_send_V6[i];
//	data_to_send_V6[_cnt++] = sum;
//	
//	HAL_UART_Transmit_DMA(&huart6,data_to_send_V6,_cnt);
//}
//ano_tc_data[0] = Vision_Info.X_Error;
//ano_tc_data[1] = gimbal.down.yaw.target_angle ;
//ano_tc_data[2] = gimbal.down.yaw.feedback_angle;
//ano_tc_data[3] = Pid_Yaw.Ep_Part;
//ano_tc_data[4] = Pid_Yaw.Ei_Part;	
//ano_tc_data[5] = Pid_Yaw.Ed_Part;
//ano_tc_data[6] = Pid_Yaw.Out;
//ano_tc_data[7] = gimbal.down.yaw.feedback_speed;
//SendToANO(ano_tc_data,sizeof(ano_tc_data));


/**
  * @brief  ����ת�����ַ���ASCii�� 
  */
uint8_t* itoa(int  num, uint8_t* str, int radix)
{
    uint8_t index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; //<������
    unsigned unum;																	  			//<���Ҫת���������ľ���ֵ,ת�������������Ǹ���
    int i=0,j,k;																					  //<i����ָʾ�����ַ�����Ӧλ��ת��֮��i��ʵ�����ַ����ĳ��ȣ�ת����˳��������ģ��������������k����ָʾ����˳��Ŀ�ʼλ��;j����ָʾ����˳��ʱ�Ľ�����
																														//<��ȡҪת���������ľ���ֵ
    if(radix==10&&num<0)																		//<Ҫת����ʮ�����������Ǹ���
    {
        unum=(unsigned)-num;																//<��num�ľ���ֵ����unum
        str[i++]='-';																				//<���ַ�����ǰ������Ϊ'-'�ţ�����������1
    }
    else unum=(unsigned)num;																//<����numΪ����ֱ�Ӹ�ֵ��unum
		
    do																							  			//<ת�����֣�ע��ת�����������
    {
        str[i++]=index[unum%(unsigned)radix];							//<ȡunum�����һλ��������Ϊstr��Ӧλ��ָʾ������1
        unum/=radix;																				//<unumȥ�����һλ
 
    }while(unum);																						//<ֱ��unumΪ0�˳�ѭ��
 
    str[i]='\0';																						//<���ַ���������'\0'�ַ���c�����ַ�����'\0'������
 
																														//<��˳���������
    if(str[0]=='-') k=1;																		//<����Ǹ��������Ų��õ������ӷ��ź��濪ʼ����
    else k=0;																								//<���Ǹ�����ȫ����Ҫ����
 
    uint8_t temp;																						//<��ʱ��������������ֵʱ�õ�
    for(j=k;j<=(i-1)/2;j++)																	//<ͷβһһ�Գƽ�����i��ʵ�����ַ����ĳ��ȣ��������ֵ�ȳ�����1
    {
        temp=str[j];																				//<ͷ����ֵ����ʱ����
        str[j]=str[i-1+k-j];																//<β����ֵ��ͷ��
        str[i-1+k-j]=temp;																	//<����ʱ������ֵ(��ʵ����֮ǰ��ͷ��ֵ)����β��
    }
    return str;																							//<����ת������ַ���
 
}
