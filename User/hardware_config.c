#include "hardware_config.h"
#include "control_logic.h"
#include "mecanum_chassis.h"
#include "motor.h"
#include "dr16.h"
#include "pid.h"
#include "holder.h"
#include "referee.h"
#include "load.h"
#include "vision.h"
#include "brain.h"
#include "Gyro.h"
#include "Supercap.h"
#include "mpu6050.h"
Holder_t Holder;
SwerveChassis swerveChassis;
Shoot_Info Shoot;
Friction_Load_Info Booster;
Chassis_Attitude_Info ChassisSwerve;
//PIDParameters swervePID =
//{
//	8,0,0,50,0,0
//};
Trace Vision_Info;//={0,0,0,0,0,0,0,0,0,0,0,0};
Motor yawMotor;

MecanumChassis chassis;

Supercap super_cap;

BasePID_Object pid_speed;

BasePID_Object pid_angle;
BasePID_Object pid_yawreset;
BasePID_Object pid_pitchreset;

BasePID_Object pid_yaw_angle;
BasePID_Object pid_yaw_speed;
BasePID_Object pid_follow;
BasePID_Object pid_base;
BasePID_Object pid_power; 
BasePID_Object pid_pitch_angle;
BasePID_Object pid_pitch_speed;
BasePID_Object pid_load;
BasePID_Object pid_friction;
BasePID_Object pid_shoot;
BasePID_Object pid_shoot1;

//uint8_t Usart5_RxBuffer[30]__attribute__((at(0x24002100)));
//uint8_t Usart5_TxBuffer[30]__attribute__((at(0x24002130)));

//UART_RxBuffer uart5_buffer={
//	.Data = Usart5_RxBuffer,
//	.Size = 30
//};
//uint8_t vofa_callback(uint8_t * recBuffer, uint16_t len)
//{
//	return 0;
//}
/**
  * @brief  初始化指令合集
  */
void HardwareConfig(void)
{
	//< 推荐先进行设备初始化，再进行硬件初始化
	MPU_Init();
	DR16Init(&rc_Ctrl);

	
	BasePID_Init(&pid_speed, 9, 0 ,0, 0);  //< 应该放在单独的源文件 parameter_config 中
	BasePID_Init(&pid_angle, 800 , 0, 300, 0);  
	BasePID_Init(&pid_follow, -50,0 ,-20 , 0);
	BasePID_Init(&pid_load,10,2 ,3, 0);
//	BasePID_Init(&pid_shoot,0, 0 ,0, 0);
	BasePID_Init(&pid_friction,10,1.5 , 2, 0);
	BasePID_Init(&pid_yawreset, 3000, 0, 550,  0);
	BasePID_Init(&pid_pitchreset, 3000, 0, 450,  0);
//	BasePID_Init(&pid_yaw_angle, -9, 0, -6,  0);  // -10 0 -10 // 235000    //-10
//	BasePID_Init(&pid_yaw_speed, 900, 0, -0,  0); // 400 0 -300  ///-300
//	BasePID_Init(&pid_pitch_angle, 6.5, 0, 3,  0); 
//	BasePID_Init(&pid_pitch_speed, 1000, 0, 400,  0); 
	BasePID_Init(&pid_yaw_angle, -5, 0, 0,  0);  //-4.5// -10 0 -10 // 235000    //-10
	BasePID_Init(&pid_yaw_speed, 2600, 0, 0,  0);   //2900
	BasePID_Init(&pid_pitch_angle, 6.5, 0, 0,  0); //11.5 // 8 0 10
	BasePID_Init(&pid_pitch_speed,1050, 0, 0,  0);//1250  //1000  0  0
  BasePID_Init(&pid_shoot, 1700, 0, 0,  0); 
	BasePID_Init(&pid_base, 1700, 0, 0,  0); 
//	BasePID_Init(&pid_power, 0, 0, 0,  0); 
	HolderInit(&Holder, pid_yaw_angle,pid_yaw_speed ,pid_pitch_angle , pid_pitch_speed ,pid_yawreset ,pid_pitchreset ,CAN1);
	SwerveChassisInit(&swerveChassis, pid_speed, pid_angle, CAN2);
	SwervePowerConrolInit(&swerveChassis, pid_base, pid_power);
	SwerveChassisSetFollowPID(&swerveChassis, pid_follow);
	LoadInit(&Shoot,pid_friction , pid_load, CAN1);

	
	
	UARTx_Init(&huart2, Brain_callback);
	UART_ENABLE_IT(&uart2, &uart2_buffer);
	
	UARTx_Init(&huart3, Referee_callback);
	UART_ENABLE_IT(&uart3, &uart3_buffer);
	
	UARTx_Init(&huart1, DR16_callback);
	UART_ENABLE_IT(&uart1, &uart1_buffer);
	
//  UARTx_Init(&huart5, vofa_callback);
//	UART_ENABLE_IT(&uart5, &uart5_buffer);
	
	
	CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CAN_Open(&can1);
	
	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can2);
	
	Gyro_Init();
	
	//IMU_Boot(&sensor,&mpuAngle,1);
	IMU_Boot(&sensor2,&mpuAngle2,2);
	
	TIMx_Init(&htim14, TIM14_Task);
	TIM_Open(&tim14);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	
  UART_Receive_DMA(&uart1, &uart1_buffer);
  UART_Receive_DMA(&uart2, &uart2_buffer);
  UART_Receive_DMA(&uart3, &uart3_buffer);
//  UART_Receive_DMA(&uart5, &uart5_buffer);
	
}

