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


void FTC_FlyControl::Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_ADC;
	

	errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10; 
	errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10; 
	errorAngle[ROLL] = applyDeadband(errorAngle[ROLL], 2);
	errorAngle[PITCH] = applyDeadband(errorAngle[PITCH], 2);
	

	Gyro_ADC = imu.Gyro_lpf / 4.0f;
	

	RateError[ROLL] = pid[PIDANGLE].get_p(errorAngle[ROLL]) - Gyro_ADC.x;
	RateError[PITCH] = pid[PIDANGLE].get_p(errorAngle[PITCH]) - Gyro_ADC.y;
	RateError[YAW] = ((int32_t)(yawRate) * rc.Command[YAW]) / 32 - Gyro_ADC.z;		
}


void FTC_FlyControl::Attitude_Inner_Loop(int32_t t,float maxacc,bool start)
{
	int32_t PIDTerm[3];
	float tiltAngle = constrain_float( max(abs(imu.angle.x), abs(imu.angle.y)), 0 ,20);
	
	for(u8 i=0; i<3;i++)
	{
		
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)	
			pid[i].reset_I();
		
		
		PIDTerm[i] = pid[i].get_pid(RateError[i], PID_INNER_LOOP_TIME*1e-6);
	}
	
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300 - abs(rc.Command[YAW]), +300 + abs(rc.Command[YAW]));	
		
	
	if(!ftc.f.ALTHOLD)
		rc.Command[THROTTLE] = (rc.Command[THROTTLE] - 1000) / cosf(radians(tiltAngle)) + 1000;
		
	if(rc.rawData[AUX1]>0 && rc.rawData[AUX1]<1300){
		if(rc.rawData[AUX2]>0 && rc.rawData[AUX2]<1300)
			{
				motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
			}
			else if(rc.rawData[AUX2]>1300 && rc.rawData[AUX2]<1600)
			{			
				if(t>0&&t<400)
					motor.writeMotor(1670, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
				else if(t>=400&&t<600)
					motor.writeMotor(1540, PIDTerm[ROLL], PIDTerm[PITCH]+80, PIDTerm[YAW]);
				else if(t>=600&&t<750)
					motor.writeMotor(1540, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
				else if(t>=750&&t<1050 )
					motor.writeMotor(1540, PIDTerm[ROLL], PIDTerm[PITCH]-80, PIDTerm[YAW]);
				else if(t>=1050&&t<1200 )
					motor.writeMotor(1540, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
				else if(t>=1200&&t<1300 )
					motor.writeMotor(1510, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
					else if(t>=1300&&t<1400 )
					motor.writeMotor(1530, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
						else if(t>=1400&&t<1500 )
					motor.writeMotor(1500, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
				else {
					ftc.f.ARMED = 0;
					motor.writeMotor(1200, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
				}			
			}
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
