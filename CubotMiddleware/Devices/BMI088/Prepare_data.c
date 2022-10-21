//#include "Prepare_data.h"
//#include "holder.h"
//#include "init_bmi088.h"
//#include "imu.h"
//#include "kalman.h"
//#include "hardware_config.h"

//float Q=0.02,R=6.00;


//float Gyro_File_Buf[3][GYRO_FILTER_NUM];
//float sumx,sumy,sumz;//sum_yaw	

//IMUAngle imuAngle;

// 	/********************************************
//	��������void PrepareForIMU()
//  ���ܣ���ȡ���������� ##��ʼ���ٶȣ����ٶȣ��¶�ʱ��
//	���룺ʹ��
//	���أ���
//	ע�⣺
//	*********************************************/
//void PrepareForIMU()
//{
//	
//	static uint8_t gyro_filter_cnt = 0;
//	int i =0;

//		bmi08a_get_data(&bmidata.acc_raw, &dev);//��ȡ���ٶ�
//		bmi08g_get_data(&bmidata.gyro_raw,&dev);//��ȡ���ٶ�


//		bmi08a_get_sensor_time(&dev,&bmidata.sensor_time);
//		bmidata.sensor_time=bmidata.sensor_time/1000;//��ȡ��ʱ�䵥λΪms	
//		sensor.acc.origin.x = bmidata.acc_raw.x;
//		sensor.acc.origin.y = bmidata.acc_raw.y;
//		sensor.acc.origin.z = bmidata.acc_raw.z;
//		
//		sensor.gyro.origin.x = bmidata.gyro_raw.x- sensor.gyro.quiet.x;
//		sensor.gyro.origin.y = bmidata.gyro_raw.y- sensor.gyro.quiet.y;
//		sensor.gyro.origin.z = bmidata.gyro_raw.z- sensor.gyro.quiet.z;
//		
//		Gyro_File_Buf[0][gyro_filter_cnt] = sensor.gyro.origin.x ;
//		Gyro_File_Buf[1][gyro_filter_cnt] = sensor.gyro.origin.y ;
//		Gyro_File_Buf[2][gyro_filter_cnt] = sensor.gyro.origin.z ;
//			
//		sumx = 0;
//		sumy = 0;
//		sumz = 0;
//		for(i=0;i<GYRO_FILTER_NUM;i++)
//		{
//			sumx += Gyro_File_Buf[0][i];
//			sumy += Gyro_File_Buf[1][i];
//			sumz += Gyro_File_Buf[2][i];
//		}

//		
//		gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
//		
//		sensor.gyro.radian.x  = sumx / (float)GYRO_FILTER_NUM * Gyro_Gr;//����/s
//		sensor.gyro.radian.y  = sumy / (float)GYRO_FILTER_NUM * Gyro_Gr;
//		sensor.gyro.radian.z  = sumz / (float)GYRO_FILTER_NUM * Gyro_Gr;
//		sensor.gyro.user.x  = sumx / (float)GYRO_FILTER_NUM;// * Gyro_Gr;  //
//		sensor.gyro.user.y  = sumy / (float)GYRO_FILTER_NUM;// * Gyro_Gr;
//		sensor.gyro.user.z  = sumz / (float)GYRO_FILTER_NUM;// * Gyro_Gr;

//		//IMU_KALMAN_Q
//		sensor.acc.averag.x =KalmanFilter_x(sensor.acc.origin.x,IMU_KALMAN_Q,IMU_KALMAN_R); //;  // ACC X�Ῠ�����˲�sensor->acc.origin.x
//		sensor.acc.averag.y =KalmanFilter_y(sensor.acc.origin.y,IMU_KALMAN_Q,IMU_KALMAN_R);//;  // ACC Y�Ῠ�����˲�sensor->acc.origin.y
//		sensor.acc.averag.z =KalmanFilter_z(sensor.acc.origin.z,IMU_KALMAN_Q,IMU_KALMAN_R); //;  // ACC Z�Ῠ�����˲�sensor->acc.origin.z


//}

///********************************************
//������void Attitude_update(void)
//���ܣ�����˫�� ���ٶ� �Ƕ� can��������Ϣ
//1��6050���ֵĵ�yaw�Ƕ�
//2����ȡYAW,Pitch��CAN�Ƕ�

// Holder.Yaw_Can_Angle   -180-+180 ˳ʱ������
// Holder.Pitch_Can_Angle    -180-+180 �����¸�

//******************************************/

//void Attitude_update(void)
//{		
//  PrepareForIMU();
//	Attitude_solution(sensor.gyro.radian.x,sensor.gyro.radian.y,sensor.gyro.radian.z,	
//	sensor.acc.averag.x,sensor.acc.averag.y,sensor.acc.averag.z);
//	Holder.Pitch.Bmi088_Angle_speed=sensor.gyro.user.y;
//	Holder.Yaw.Bmi088_Angle_speed=sensor.gyro.user.z;
////	imuAngle.YawAngleSpeed = sensor.gyro.user.z;
//if(abs(Holder.Yaw.Bmi088_Angle_speed)<GYRO_GAP)
//	{
//		Holder.Yaw.Bmi088_Angle_speed=0;
//	}		
////	if(abs(imuAngle.YawAngleSpeed) < GYRO_GAP)
////	{
////		imuAngle.YawAngleSpeed=0;
////	}
////	imuAngle.Yaw -= (imuAngle.YawAngleSpeed) * K_ANGLESPEED_2_ANGLE * 0.458f;
//			Holder.Yaw.Bmi088_Angle-=(Holder.Yaw.Bmi088_Angle_speed)*K_ANGLESPEED_2_ANGLE;
//	 Holder.Pitch.Bmi088_Angle =-angle.pitch; 
////			Slave_Board.Slave_Yaw -=(Slave_Board.Slave_yaw_speed)*K_ANGLESPEED_2_ANGLE*0.15266f;
//}
//	
// 