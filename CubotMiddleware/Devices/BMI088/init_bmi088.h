#ifndef _INIT_BMI088_H_     //���û�ж��������
#define _INIT_BMI088_H_   

#include "bmi08x.h"
#include "bmi088.h"
#include "spi_bmi088.h"//????

//#define Acc_G 		0.0011963f		//  1/32768/4/9.8     ���ٶ�����Ϊ4G		
#define Gyro_G 		0.030517578125l	//  1/32768*1000      ����������Ϊ +��1000			
#define Gyro_Gr		(0.0005325929864746946l*2)  //  1/32768*1000/57.3 
#define abs(x) ((x)>0? (x):(-(x)))


struct _float{
	      float x;
				float y;
				float z;};

struct _double{
       double x;
	     double y;
	     double z;};	

struct _int16_t{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _float origin;  //ԭʼֵ
	   struct _float averag;  //ƽ��ֵ
	   struct _float histor;  //��ʷֵ
	   struct _float quiet;   //��ֵ̬
	   struct _float radian;  //����ֵ 
		 struct _float user;//δ��adcת��ֵ
          };
/**
	*@biref SENSOR_StoreTypeDef#���������ݽṹ�壬���ڽ���ȡ�����ݽ�������
					*/
typedef struct {   
	struct _trans acc;
	struct _trans gyro;
}SENSOR_StoreTypeDef;
/**
	*@biref BMI_DataTypeDef#���������ݽṹ�壬���ڻ�ȡ����
					*/
typedef struct {
	struct bmi08x_sensor_data acc_raw;
	struct bmi08x_sensor_data gyro_raw;
	int sensor_temp;
	uint32_t sensor_time;
}BMI_DataTypeDef;													
				


//bmi088 part			to out				

 int8_t bmi08a_get_data(struct bmi08x_sensor_data *accel,const struct bmi08x_dev *dev);
 int8_t bmi08g_get_data(struct bmi08x_sensor_data *gyro,const struct bmi08x_dev *dev);
int8_t bmi08x_start(struct bmi08x_dev *dev);

void Gyro_OFFEST(void);
void First_Gyro_OFFEST(void);
void BMI088_Init(void);

extern SENSOR_StoreTypeDef sensor;						
extern BMI_DataTypeDef bmidata;	
extern struct bmi08x_dev dev;

#endif											
							/**/
							
							
							
							
							
