#ifndef DRV_USART_H
#define DRV_USART_H
#include "stm32h7xx_hal.h"
#include "usart.h"


/**
  * @brief   �����û��ص���������
  * @param[in]  rxBuffer		���ڽ������ݵ������׵�ַ
  * @param[in]  size	    	���ڽ������ݵ����鳤��
  */
typedef uint8_t (*UART_RxIdleCallback)(uint8_t *rxBuffer, uint16_t size); ///<�൱��������һ������ָ������


/**
	* @brief	UART���ͻ�����
	*/
typedef struct
{
  uint8_t* Data;
	uint16_t Size;
}UART_RxBuffer;


/**
	* @brief	UART���ջ�����
	*/
typedef struct
{
	uint8_t* Data;
	uint16_t DataSize;
}UART_TxBuffer;


/**
  * @brief   ���ڽṹ��,��������ͽ��ջص�
  */
typedef struct 
{
	UART_HandleTypeDef* Handle;
	UART_RxIdleCallback RxIdleCallback;        
}UART_Object;


/**
  * @brief   ���ڳ�ʼ����������ͽ��ջص����������ڽṹ��
  * @param[in]  handle		        ���ھ��
  * @param[in]  rxIdleCallback		���ջص�����
  */
void UARTx_Init(UART_HandleTypeDef* handle, UART_RxIdleCallback rxIdleCallback);


/**
  * @brief  ���ڹ������ṹ������Ѿ�Ԥ����д�õĴ����豸��ʼ��
  * @param[in]  uart		    ���ڽṹ��, �������ھ��
  * @param[in]  rxBuffer		���ջ�����,�û�����
  * @retval 
  */
void UART_Receive_DMA(UART_Object* uart, UART_RxBuffer* rxBuffer);
void UART_ENABLE_IT(UART_Object* uart, UART_RxBuffer* rxBuffer);

/**
  * @brief  ���ڵ���dma�������ݣ���������Ϊ�ֽ�
  * @param[in]  uart		    ���ڽṹ�壬����ʹ�õĴ��ں�
  * @param[in]  txBuffer		���ͻ�����.�û�����
  * @retval 
  */
uint32_t UART_Send(UART_Object* uart, UART_TxBuffer* txBuffer);


/**
  * @brief  �����豸�жϺ�����ִ���ж�DMA���������ô����û��ص���������Ҫ������ӵ�stm32h7xx_it.c��
  * @param[in]  uart		    ���ڽṹ��, �������ھ���ͻص�����
  * @param[in]  rxBuffer		���ͻ�����, �û����塣ע�⣡ �˴���rxBufferӦ���� UART_Open�е���rxBufferһ��
  * @retval 
  */
void UART_Idle_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer);


/**
  * @brief  ����ת�����ַ���ASCii��
  * @param  num ������ radix����������
  * @retval str �ַ���
  */
uint8_t* itoa(int num,uint8_t* str, int radix);
void ANO_V6_Send_Up_Computer(int16_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6);
extern uint8_t data_to_send_V6[20]__attribute__((at(0x24002000)));

extern UART_Object uart1;
extern UART_Object uart2;
extern UART_Object uart3;
extern UART_Object uart4;
extern UART_Object uart5;
extern UART_Object uart6;


#endif 


