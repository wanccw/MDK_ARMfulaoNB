#include "heat_control.h"
#include "referee.h"
 Heat_Info Muzzle=
 {
	 .cooling_times=1
 };
 /*1ms执行1次*/
 /*
 裁判系统扣血逻辑
     设定机器人的枪口热量上限为 Q0，当前枪口热量为 Q1， 裁判系统每检测到一发 17mm 弹丸， 当前枪口热
 量 Q1 增加 10（与 17mm 弹丸的初速度无关）。
			若 Q1 > Q0，该机器人对应操作手电脑的第一视角可视度降低。直到 Q1 < Q0，第一视角才会恢复正
 常
      若 2Q0 > Q1 > Q0，每 100 ms 扣除血量 = ((Q1 - Q0) / 250) / 10 * 上限血量
			若 Q1 ≥ 2Q0，立刻扣除血量 = (Q1 - 2Q0) / 250 * 上限血量。扣血后令 Q1 = 2Q0。
 */
 
void Heat_Control()
{ 
//	if(referee2022.power_heat_data.renewshooter_heat0==0)//裁判系统热量不更新，自行冷却
//	{		
//			//Muzzle.on_time_heat-=Get_Heat_Cold_Speed(game_robot_state.robot_level)*Muzzle.cooling_times/1000.0f;//
//	  	Muzzle.on_time_heat-=referee2022.game_robot_status.shooter_id1_17mm_cooling_rate*Muzzle.cooling_times/1000.0f; //执行周期为1kHz
//		
//	}
	 if(referee2022.power_heat_data.renewshooter_heat0==1)// 裁判系统热量信息更新了 重新给当前热量赋值
	{
		  Muzzle.on_time_heat=referee2022.power_heat_data.shooter_id1_17mm_cooling_heat;//裁判系统热量修正
//		  referee2022.power_heat_data.shooter_heat0=0;  //未更新
	}
	
	if(Muzzle.on_time_heat<0)
	{
		 Muzzle.on_time_heat=0;  //
	}
	if(Muzzle.over_heat==1)//超热量
	{
		Muzzle.on_time_heat=0;
	}
}
uint16_t Get_Max_Heat(uint8_t Robot_Level)//获取枪口热量上限
{
	if      (Robot_Level==1)    return 120;	//
	else if (Robot_Level==2)    return 240;				
	else if (Robot_Level==3)    return 360;	

	//我猜是这个
	
	return referee2022.power_heat_data.shooter_id1_17mm_cooling_heat;
}
uint16_t Get_Heat_Cold_Speed(uint8_t Robot_Level)//获取枪口热量冷却速度（单发子弹速度上限为30m/s）
{
	if      (Robot_Level==1)    return 20;	//枪口每秒冷却 30 40 50	血量低于20%，冷却速度x2
	else if (Robot_Level==2)    return 40;				
	else if (Robot_Level==3)    return 60;	
	return 0;
}