#ifndef DRV_IIC_H
#define DRV_IIC_H
#include "stm32h7xx_hal.h"
#include "i2c.h"
#include "driver_iic.h"

#define I2Cx_FLAG_TIMEOUT             ((uint32_t) 1000) //0x1100
#define I2Cx_LONG_TIMEOUT             ((uint32_t) (300 * I2Cx_FLAG_TIMEOUT)) //was300


/**
  * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
	* @param  slave_addr: �ӻ���ַ
	* @param 	reg_addr:�Ĵ�����ַ
	* @param len��д��ĳ���
	*	@param data_ptr:ָ��Ҫд�������
  * @retval ����Ϊ0��������Ϊ��0
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char* data_ptr);

	int Sensors_I2C_WriteRegister2(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char* data_ptr);
	
/**
  * @brief  ���Ĵ����������ṩ���ϲ�Ľӿ�
	* @param  slave_addr: �ӻ���ַ
	* @param 	reg_addr:�Ĵ�����ַ
	* @param len��Ҫ��ȡ�ĳ���
	*	@param data_ptr:ָ��Ҫ�洢���ݵ�ָ��
  * @retval ����Ϊ0��������Ϊ��0
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char* data_ptr);
	
	int Sensors_I2C_ReadRegister2(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char* data_ptr);

#endif 

