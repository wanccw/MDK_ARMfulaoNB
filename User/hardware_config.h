#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_
#include "stm32h7xx_hal.h"
#include "driver.h"
#include "swerve_chassis.h"
#include "mecanum_chassis.h"
#include "motor.h"
#include "holder.h"
#include "load.h"
#include "vision.h"
#include "Supercap.h"
#include "brain.h"
/**
  * @brief  完成剩余的驱动器配置，开启驱动器
  */
void HardwareConfig(void);
void DR16_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer, uint16_t len , Chassis_Attitude_Info* Swerve , Trace* Info_Vision , SwerveChassis* chassis , Shoot_Info* shoot );
void KeyBoard_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer, uint16_t len , Chassis_Attitude_Info* Swerve , Trace* Info_Vision , SwerveChassis* chassis , Shoot_Info* shoot ,Supercap* Cap,CubotBrain_t* brain);
void Chassis_Mode_Control(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Holder_t* holder , Chassis_Attitude_Info* Swerve,Supercap* Cap);
uint8_t Supercap_rxCallBack(CAN_Object canx, CAN_RxBuffer rxBuffer , Supercap* Cap); 
void SupercapControl(CAN_Object can, RC_Ctrl* rc_ctrl,Supercap* Cap);
extern SwerveChassis swerveChassis;

extern PIDParameters swervePID;

extern Motor yawMotor;

extern MecanumChassis chassis;

extern BasePID_Object pid_speed;

extern BasePID_Object pid_angle;

extern BasePID_Object pid_yaw;

extern BasePID_Object pid_shoot;
extern BasePID_Object pid_shoot1;

extern UART_RxBuffer uart1_buffer;

extern BasePID_Object pid_follow;

extern Chassis_Attitude_Info Swerve;

extern Trace Vision_Info;

extern Friction_Load_Info Booster;

extern Shoot_Info Shoot;

extern  Holder_t Holder;

extern Chassis_Attitude_Info ChassisSwerve;

extern Supercap super_cap;

extern UART_RxBuffer uart5_buffer;
#endif



