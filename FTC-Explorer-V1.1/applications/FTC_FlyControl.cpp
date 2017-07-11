/******************** (C) COPYRIGHT 2015 FTC ***************************
 * 作者		 ：FTC
 * 文件名  ：FTC_FlyControl.cpp
 * 描述    ：飞行控制
**********************************************************************************/
#include "FTC_FlyControl.h"


FTC_FlyControl fc;

FTC_FlyControl::FTC_FlyControl()
{
	rollPitchRate = 150;
	yawRate = 50;
	
	altHoldDeadband = 100;
	
	//重置PID参数
	PID_Reset();
}

//重置PID参数
void FTC_FlyControl::PID_Reset(void)
{
	pid[PIDROLL].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDPITCH].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDYAW].set_pid(0.8, 0.45, 0, 200);
	pid[PIDANGLE].set_pid(5, 0, 0, 0);
	pid[PIDMAG].set_pid(2, 0, 0, 0);
 	pid[PIDVELZ].set_pid(1.5, 0.5, 0.002, 150);
 	pid[PIDALT].set_pid(1.2, 0, 0, 200);
}

int32_t yaw_pid,roll_pid,pitch_pid;
int32_t x_pid,y_pid,z_pid;
//飞行器姿态外环控制
void FTC_FlyControl::Attitude_Outter_Loop(void)
{
	//to do
	fc.PID_Reset();
	roll_pid=pid[PIDROLL].get_pid(rc.Command[ROLL]-imu.angle.x,PID_OUTER_LOOP_TIME*1e-6);
	pitch_pid=pid[PIDPITCH].get_pid(rc.Command[PITCH]-imu.angle.y,PID_OUTER_LOOP_TIME*1e-6);
	yaw_pid=pid[PIDYAW].get_pid(rc.Command[YAW]-imu.angle.z,PID_OUTER_LOOP_TIME*1e-6);
}

//飞行器姿态内环控制
void FTC_FlyControl::Attitude_Inner_Loop(void)
{
	//to do
	if(rc.Command[THROTTLE]>RC_MINTHROTTLE)
	{
		x_pid=pid[PIDROLL].get_pid(roll_pid-imu.Gyro.x,PID_INNER_LOOP_TIME*1e-6);
		y_pid=pid[PIDPITCH].get_pid(pitch_pid-imu.Gyro.y,PID_INNER_LOOP_TIME*1e-6);
		z_pid=pid[PIDYAW].get_pid(yaw_pid-imu.Gyro.z,PID_INNER_LOOP_TIME*1e-6);
		int16_t thro;
		thro=rc.Command[THROTTLE]/cosf(radians(max(abs(imu.angle.x),abs(imu.angle.y))));
		motor.writeMotor(thro,x_pid,y_pid,z_pid);
	}
	else
	{
		pid[PIDROLL].reset_I();
		pid[PIDPITCH].reset_I();
		pid[PIDYAW].reset_I();
	}
}	

//飞行器高度外环控制
void FTC_FlyControl::Altitude_Outter_Loop(void)
{
	//to do
}

//飞行器高度内环控制
void FTC_FlyControl::Altitude_Inner_Loop(void)
{
	//to do
}

void FTC_FlyControl::AltHoldReset(void)
{
	AltHold = nav.position.z;
}

/************************ (C) COPYRIGHT 2015 FTC *****END OF FILE**********************/
