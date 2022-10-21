/**@file  bsp_usart.c
* @brief    �弶֧�ְ������ڹ����������ļ����û��ص��ض���
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
*/
#include "drv_usart.h"


struct uart_manage_obj uart_1_manage=
{
	.uart_idle_callback = NULL
};

struct uart_manage_obj uart_2_manage=
{
	.uart_idle_callback = NULL
};


uint8_t uart_manage_init(uart_manage_obj *uart_manage,UART_HandleTypeDef *huart,uint8_t *Rx_Buffer,uint8_t Rec_Buffer_Size)
{
	uart_manage->huart=(*huart);
	uart_manage->Rec_Buffer=Rx_Buffer;
	uart_manage->Rec_BUFFER_SIZE=Rec_Buffer_Size;
	
	HAL_UART_Receive_DMA(&uart_manage->huart,uart_manage->Rec_Buffer,uart_manage->Rec_BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&uart_manage->huart, UART_IT_IDLE);  //<�����ж�һ��Ҫ���ں���ָ�븳ֵ���֮�󣬷������ָ��NULL������hard_fault();��ָ��NULL�������ã�����Ұָ����Ѿ������ˣ�

    return 1;
} 

uint8_t uart_callback_register(uart_manage_obj *uart_manage,uart_stdmsg_callback_t rx_fn)
{
	if(rx_fn == NULL)
		return 1;
	else 
		uart_manage->uart_idle_callback = rx_fn;
	return 0;
}

void UART_IDLE_CALLBACK(uart_manage_obj *uart_manage)  
{
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET))
	{  
		HAL_UART_DMAStop(&uart_manage->huart);
		__HAL_UART_CLEAR_IDLEFLAG(&uart_manage->huart);
//		if ((uart_manage->Rec_BUFFER_SIZE - uart_manage->huart.hdmarx->Instance->NDTR) == uart_manage->Data_len)
		{
			if((*uart_manage).uart_idle_callback!=NULL)
				uart_manage->uart_idle_callback(uart_manage->Rec_Buffer,uart_manage->Rec_BUFFER_SIZE); //<�û��ص�
		}

//	Offline_Check.Last_Update.Dbus=System.Sys_time;
		HAL_UART_Receive_DMA(&uart_manage->huart,uart_manage->Rec_Buffer,uart_manage->Rec_BUFFER_SIZE);
	}
}

/**
  * @brief ң�������տ����жϻص�
  * @param uart_manage:
  * @retval 
	* @usedas:
						uart1_idle_callback( task_t remote_deal,NULL,Remoter_USART1_Rec_Buffer, 30);
  */
//uint32_t uart1_idle_callback(uint8_t *buf, uint32_t Rec_Buffer_Size)
//{
//	uint8_t rx_len=0;
//	uint32_t tmp_flag = 0;
//	uint32_t temp;
//	tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //��ȡIDLE��־λ
//	
//	if((tmp_flag != RESET))//idle��־����λ
//	{ 
//		{
//			__HAL_UART_CLEAR_IDLEFLAG(&huart1);//�����־λ
//			HAL_UART_DMAStop(&huart1); //
//			temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// ��ȡDMA��δ��������ݸ���   
//			rx_len =  Rec_Buffer_Size - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
//		}
//		
////		remote_deal(buf,0);  //		struct Return_Struct* Remote_deal(uint8_t * Remoter_USART1_Rec_Buffer );
////		(*(uart_1_manage.task1))(buf,0);
//		if(uart_1_manage.uart_idle_callback!=NULL)
//		{
//				uart_1_manage.uart_idle_callback(buf,0);//���������remote_deal(buf,0);�ĸ������㻹�д�˼��----����ʵ�Ҹ�������remote_deal�������ٴ��֣�����ֱ�ۡ�����ֻ��Ϊ��ʵ��̽��һ�²��õ���һ�С�
//		}
//		
//		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf,Rec_Buffer_Size);			
//	  rx_len = 0;//�������
//	  HAL_UART_Receive_DMA(&huart1,buf , Rec_Buffer_Size);
//	 }
//	
//    //.......
//    return 1;
//}

void	UART1_IDLE_CALLBACK(uart_manage_obj *uart_manage)
{
	uint8_t rx_len=0;
	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //��ȡIDLE��־λ
	
	if((tmp_flag != RESET))//idle��־����λ
	{ 
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);//�����־λ
			HAL_UART_DMAStop(&huart1); //
			temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// ��ȡDMA��δ��������ݸ���   
			rx_len =  uart_manage->Rec_BUFFER_SIZE - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		}
		
//		remote_deal(buf,0);  //		struct Return_Struct* Remote_deal(uint8_t * Remoter_USART1_Rec_Buffer );
//		(*(uart_1_manage.task1))(buf,0);
		if(uart_1_manage.uart_idle_callback!=NULL)
		{
				uart_1_manage.uart_idle_callback(uart_manage->Rec_Buffer,0);//���������remote_deal(buf,0);�ĸ������㻹�д�˼��----����ʵ�Ҹ�������remote_deal�������ٴ��֣�����ֱ�ۡ�����ֻ��Ϊ��ʵ��̽��һ�²��õ���һ�С�
		}
		
		HAL_UART_Transmit_DMA(&huart2, uart_manage->Rec_Buffer,uart_manage->Rec_BUFFER_SIZE);			
	  rx_len = 0;//�������
	  HAL_UART_Receive_DMA(&huart1,uart_manage->Rec_Buffer , uart_manage->Rec_BUFFER_SIZE);
	 }
	
    //.......
 
}




















/**
  * @brief USART2�����ж�
  * @param uart_manage:
  * @retval 
	* @usedas:
						uart1_idle_callback( task_t remote_deal,NULL,Remoter_USART1_Rec_Buffer, 30);

  */

volatile uint8_t rx_len = 0;  //����һ֡���ݵĳ���
volatile uint8_t recv_end_flag = 0; //һ֡���ݽ�����ɱ�־
uint8_t rx_buffer[100]={0};  //�������ݻ�������

uint32_t uart2_idle_callback(uint8_t *buf, uint32_t Rec_Buffer_Size)
{

	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); //��ȡIDLE��־λ
	
	if((tmp_flag != RESET))//idle��־����λ
	{ 
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);//�����־λ
			HAL_UART_DMAStop(&huart2); //
			temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);// ��ȡDMA��δ��������ݸ���   
			rx_len =  Rec_Buffer_Size - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		}
		
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf,rx_len);			
	  rx_len = 0;//�������
	  HAL_UART_Receive_DMA(&huart2,buf , 100);

	 }
	
    //.......
    return 1;
}


 

/**
  * @brief turn the 
  * @param uart_manage:
  * @retval 
	* @usedas:

  */
uint8_t* itoa(int  num,uint8_t* str,int radix)
{
    uint8_t index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//������
    unsigned unum;//���Ҫת���������ľ���ֵ,ת�������������Ǹ���
    int i=0,j,k;//i����ָʾ�����ַ�����Ӧλ��ת��֮��i��ʵ�����ַ����ĳ��ȣ�ת����˳��������ģ��������������k����ָʾ����˳��Ŀ�ʼλ��;j����ָʾ����˳��ʱ�Ľ�����
 
    //��ȡҪת���������ľ���ֵ
    if(radix==10&&num<0)//Ҫת����ʮ�����������Ǹ���
    {
        unum=(unsigned)-num;//��num�ľ���ֵ����unum
        str[i++]='-';//���ַ�����ǰ������Ϊ'-'�ţ�����������1
    }
    else unum=(unsigned)num;//����numΪ����ֱ�Ӹ�ֵ��unum
 
    //ת�����֣�ע��ת�����������
    do
    {
        str[i++]=index[unum%(unsigned)radix];//ȡunum�����һλ��������Ϊstr��Ӧλ��ָʾ������1
        unum/=radix;//unumȥ�����һλ
 
    }while(unum);//ֱ��unumΪ0�˳�ѭ��
 
    str[i]='\0';//���ַ���������'\0'�ַ���c�����ַ�����'\0'������
 
    //��˳���������
    if(str[0]=='-') k=1;//����Ǹ��������Ų��õ������ӷ��ź��濪ʼ����
    else k=0;//���Ǹ�����ȫ����Ҫ����
 
    uint8_t temp;//��ʱ��������������ֵʱ�õ�
    for(j=k;j<=(i-1)/2;j++)//ͷβһһ�Գƽ�����i��ʵ�����ַ����ĳ��ȣ��������ֵ�ȳ�����1
    {
        temp=str[j];//ͷ����ֵ����ʱ����
        str[j]=str[i-1+k-j];//β����ֵ��ͷ��
        str[i-1+k-j]=temp;//����ʱ������ֵ(��ʵ����֮ǰ��ͷ��ֵ)����β��
    }
 
    return str;//����ת������ַ���
 
}


