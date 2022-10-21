#ifndef DRV_USART_H
#define DRV_USART_H
#include "stm32h7xx_hal.h"
#include "usart.h"



/**
  * @brief   �����û��ص���������
  */
typedef uint8_t (*uart_stdmsg_callback_t)(uint8_t *buff , uint16_t len);//�൱��������һ������ָ������

/**
  * @brief   ���ڹ������ṹ��
  */
typedef struct uart_manage_obj
{
    UART_HandleTypeDef huart;  
    uint8_t *Rec_Buffer;
    uint16_t  Rec_BUFFER_SIZE;
	  uint16_t Data_len;
    uart_stdmsg_callback_t uart_idle_callback;
}uart_manage_obj; 

/**
  * @brief  �����豸��ʼ����ͨ�����ڹ�����ִ�г�ʼ����ֵ 
  * @param uart_manage:
  * @retval 
  */
uint8_t uart_manage_init(uart_manage_obj *uart_manage,UART_HandleTypeDef *huart,uint8_t *Rx_Buffer,uint8_t Rec_Buffer_Size);

/**
  * @brief  �����û��ص�������ʼ�� �����Ĵ��ڹ������лص�����ָ��ĸ�ֵ
  * @param uart_manage:
  * @retval 
  */
uint8_t uart_callback_register(uart_manage_obj *uart_manage,uart_stdmsg_callback_t rx_fn);

/**
  * @brief  �����豸�жϺ�����ִ���ж�DMA���������ô����û��ص����� 
  * @param uart_manage:
  * @retval 
  */
void UART_IDLE_CALLBACK(uart_manage_obj *uart_manage);

/**
  * @brief  ����ת�����ַ���ASCii��
  * @param  num ������ radix����������
  * @retval str �ַ���
  */
uint8_t* itoa(int num,uint8_t* str,int radix);




void	UART1_IDLE_CALLBACK(uart_manage_obj *uart_manage);




extern struct uart_manage_obj uart_1_manage;
extern struct uart_manage_obj uart_2_manage;


#endif 

