/**@file Gyro.c
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

/**********************************************************************************
 ==============================================================================
						  How to use this driver
 ==============================================================================

	��� MPU6050.h

	1. ����MPU6050_Init()
  2. �� Gyro_Get_Data() ����1ms���ж�����
  

  ********************************************************************************/


#include "Gyro.h"
#include "mpu6050.h"
//#include "filter.h"
//#include "task.h"
#include "driver_timer.h"
#include "kalman.h"
#include "holder.h"
#include "hardware_config.h"
#include "filter.h"

//>���ڴ���1����
gyro_data_t  gyro_data=
{
	.sens_yaw  =0.499f,  //0.3   //0.6783, // yaw   ���ٶȻ���Ϊ�Ƕȵ�ֵ  ����ģ��ʹ��
	.sens_pitch=0.6783,  // pitch ���ٶȻ���Ϊ�Ƕȵ�ֵ  δ��
  .cqhlp = 10
};

gyro_data_t  gyro_data2=
{
	.sens_yaw  =0.4216,//0.479f,     //0.6783, // yaw   ���ٶȻ���Ϊ�Ƕȵ�ֵ   δ��
	.sens_pitch=0.3961,  // pitch ���ٶȻ���Ϊ�Ƕȵ�ֵ   ����ģ��ʹ��      1.3269
  .cqhlp = 10
};

void MPU_Init(void)
{
	//MUP structure variable define
	MPU_Region_InitTypeDef MPU_Config;
	
	/*-----------Open FPU--------*///High speed FLOAT calculate
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
	/*-----------Open Cache------------*/
	SCB_EnableICache();//ʹ��I-Cache
  SCB_EnableDCache();//ʹ��D-Cache   
	SCB->CACR|=1<<2;   //ǿ��D-Cache͸д,�粻����,ʵ��ʹ���п���������������	
	/*-----------Open MPU------------*/
	HAL_MPU_Disable();
	
	MPU_Config.Enable=MPU_REGION_ENABLE;
	MPU_Config.Number=MPU_REGION_NUMBER1;//��������� 1
	MPU_Config.BaseAddress= 0x24000000;//����������ַ
	MPU_Config.Size=MPU_REGION_SIZE_512KB;//���ñ�����512k
	MPU_Config.SubRegionDisable=0x00;//��ֹ������
	MPU_Config.TypeExtField=MPU_TEX_LEVEL0;//����������չ��Ϊlevel0
	MPU_Config.AccessPermission=MPU_REGION_FULL_ACCESS;//ȫ���ʣ���Ȩ&�û����ɷ��ʣ�
	MPU_Config.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;//����ָ�����
	MPU_Config.IsShareable=MPU_ACCESS_SHAREABLE;//������
	MPU_Config.IsCacheable=MPU_ACCESS_CACHEABLE;//����cache
	MPU_Config.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;//�������� DMAģʽ��Ҫ��Ϊ������
	HAL_MPU_ConfigRegion(&MPU_Config);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}	


void Gyro_Init(void)
{
	//>�ϵ�
		MPU6050_PowerOn();
	//>��ʼ��
	  //MPU6050_Init();
		MPU6050_Init2();
	//>����Ϊ 0
	  gyro_data.cnt=0;
		gyro_data2.cnt=0;

}


void Gyro_Reset(void)
{
	//>�µ�
	  MPU6050_PowerOff();
	  HAL_Delay(50);
	
	//>�ϵ��ʼ��
    Gyro_Init();

}

/*
�������� 
yaw��̬���ٶ�   28
pitch��̬���ٶ� 26


*/
//>����1ms�Ķ�ʱ���ж���
float yaw_data_filter_mpu[5]={0};
float pitch_data_filter_mpu[5]={0};
int8_t yaw_filter_mpu_cnt=0;
int8_t pitch_filter_mpu_cnt=0;
void Gyro_Get_Data(void){
	//>���ݸ��´����ۼ�
  gyro_data.cnt++;
	
  //>MPU6050���ݸ���
	MPU6050_RawDataUpdate(&mpuAngle,1);

if(gyro_data.cnt>1000){	
		//>��Ԫ������� pitch�Ƕ�
	
	//>�ǶȻ�Ϊ����  ���ٶȽ��п������˲�  ����һ���޸ģ�
	PrepareForIMU(&sensor,&mpuAngle);
	IMUupdate(&sensor,&mpuAngle,1);
	
	gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z-(int16_t)sensor.gyro.quiet.z;
		//gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z+70;
	//gyro_data.gyro_w_yaw   = sensor.gyro.averag.z;
	//F_ar5_filter(gyro_data.gyro_w_yaw,yaw_data_filter_mpu,&yaw_filter_mpu_cnt);
	
	gyro_data.gyro_w_pitch = mpuAngle.gyroRaw.x-(int16_t)sensor.gyro.quiet.x;
	//gyro_data.gyro_w_pitch = sensor.gyro.averag.x;
	//F_ar5_filter(gyro_data.gyro_w_pitch,pitch_data_filter_mpu,&pitch_filter_mpu_cnt);
		//gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z-37;

	if(abs(gyro_data.gyro_w_yaw)>100){
			gyro_data.yaw_speed=((gyro_data.gyro_w_yaw)*0.0001*gyro_data.sens_yaw);
			gyro_data.yaw_angle= gyro_data.yaw_angle + gyro_data.yaw_speed;}
		
	if(abs(gyro_data.gyro_w_pitch)>100){		
			gyro_data.pitch_speed = ((gyro_data.gyro_w_pitch)*0.01f*gyro_data.sens_pitch);
	    gyro_data.pitch_angle= gyro_data.pitch_angle + gyro_data.pitch_speed;}

	if(tim14.HolderTime < 3500){
		gyro_data.yaw_angle=0;}
	}
}

void Gyro_Get_Data2(void){
	//>���ݸ��´����ۼ�
  gyro_data2.cnt++;
	
  //>MPU6050���ݸ���
	MPU6050_RawDataUpdate(&mpuAngle2,2);

if(gyro_data2.cnt>1000){	

	//>�ǶȻ�Ϊ����  ���ٶȽ��п������˲�  ����һ���޸ģ�
	PrepareForIMU(&sensor2,&mpuAngle2);
		//>��Ԫ������� pitch�Ƕ�
	IMUupdate(&sensor2,&mpuAngle2,2);	
	
	gyro_data2.gyro_w_yaw   = mpuAngle2.gyroRaw.z-(int16_t)sensor2.gyro.quiet.z;
		//gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z+70;
	//gyro_data.gyro_w_yaw   = sensor.gyro.averag.z;
	//F_ar5_filter(gyro_data.gyro_w_yaw,yaw_data_filter_mpu,&yaw_filter_mpu_cnt);
	
	gyro_data2.gyro_w_pitch = mpuAngle2.gyroRaw.x-(int16_t)sensor2.gyro.quiet.x;
	//gyro_data.gyro_w_pitch = sensor.gyro.averag.x;
	//F_ar5_filter(gyro_data.gyro_w_pitch,pitch_data_filter_mpu,&pitch_filter_mpu_cnt);
		//gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z-37;

	if(abs(gyro_data2.gyro_w_yaw)>10){
			gyro_data2.yaw_speed=((gyro_data2.gyro_w_yaw)*0.0001*gyro_data2.sens_yaw);
			gyro_data2.yaw_angle= gyro_data2.yaw_angle + gyro_data2.yaw_speed;}
		
	if(abs(gyro_data2.gyro_w_pitch)>10){		
			gyro_data2.pitch_speed = ((gyro_data2.gyro_w_pitch)*0.0001f*gyro_data2.sens_pitch);
	    gyro_data2.pitch_angle= gyro_data2.pitch_angle + gyro_data2.pitch_speed;}
		}
}

void MPU_Get_Data(Holder_t* holder)
{
	//Gyro_Get_Data();
	Gyro_Get_Data2();
	holder->Yaw.MPU6050_Angle=-gyro_data2.yaw_angle;
	holder->Yaw.MPU6050_Angle_speed=gyro_data2.yaw_speed;
  holder->Yaw.MPU6050_Angle_speed1=holder->Yaw.MPU6050_Angle_speed*150;
//holder->Yaw.MPU6050_Angle_speed1=((int)(gyro_data.yaw_speed*150*10))*0.1;
//holder->Yaw.MPU6050_Angle_speed1=LPFilter(holder->Yaw.MPU6050_Angle_speed1 ,&LPF_yaw_mpu);
	holder->Pitch .MPU6050_Angle=gyro_data2.pitch_angle;
	holder->Pitch.MPU6050_Angle_speed=gyro_data2.pitch_speed;
	holder->Pitch.MPU6050_Angle_speed1=holder->Pitch.MPU6050_Angle_speed*150;
//holder->Pitch.MPU6050_Angle_speed1=((int)(gyro_data2.pitch_speed*150*10))*0.1;
//holder->Pitch.MPU6050_Angle_speed1=LPFilter(holder->Pitch.MPU6050_Angle_speed1 ,&LPF_pitch_mpu);
}

