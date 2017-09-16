/******************** (C) COPYRIGHT 2015 FTC ***************************
 * 作者		 ：FTC
 * 文件名  ：FTC_Motor.cpp
 * 描述    ：电机控制相关函数
**********************************************************************************/
#include "FTC_Motor.h"

FTC_Motor motor;

void FTC_Motor::writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	//六轴X型
	motorPWM[2] = throttle - 0.5 * pidTermRoll + 0.866 *  pidTermPitch + pidTermYaw; //后右
	motorPWM[1] = throttle - 0.5 * pidTermRoll - 0.866 *  pidTermPitch + pidTermYaw; //前右
	motorPWM[0] = throttle + 0.5 * pidTermRoll + 0.866 *  pidTermPitch - pidTermYaw; //后左
	motorPWM[3] = throttle + 0.5 * pidTermRoll - 0.866 *  pidTermPitch - pidTermYaw; //前左
	motorPWM[5] = throttle - pidTermRoll - pidTermYaw;	//右
	motorPWM[4] = throttle + pidTermRoll + pidTermYaw;	//左
	
	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//限制电机PWM的最小和最大值
		motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}

	//如果未解锁,则将电机输出设置为最低
	
	if(!ftc.f.ARMED)	
		ResetPWM();

	if(!ftc.f.ALTHOLD && rc.rawData[THROTTLE] < RC_MINCHECK)
		ResetPWM();

	
	//抛飞.Version1
	
	if (ftc.f.ARMED) {
		if (imu.Acc.z > 2*ACC_1G || flag == 0) {
			if (time < 3000) {
				rc.jyszz = 0;
				
				uint16_t throttle_2 = 1800;
				
				if (time >= 1000 && time <3000) 
					throttle_2 = throttle_2 - 30 * (time / 500 - 2);
				
				motorPWM[2] = throttle_2 - 0.5 * pidTermRoll + 0.866 *  pidTermPitch + pidTermYaw; //后右
				motorPWM[1] = throttle_2 - 0.5 * pidTermRoll - 0.866 *  pidTermPitch + pidTermYaw; //前右
				motorPWM[0] = throttle_2 + 0.5 * pidTermRoll + 0.866 *  pidTermPitch - pidTermYaw; //后左
				motorPWM[3] = throttle_2 + 0.5 * pidTermRoll - 0.866 *  pidTermPitch - pidTermYaw; //前左
				motorPWM[5] = throttle_2 - pidTermRoll - pidTermYaw;	//右
				motorPWM[4] = throttle_2 + pidTermRoll + pidTermYaw;	//左
				time++;
				flag = 0;
			}
			
			if (time >= 3000) {
				flag = 1;
				time = 0;
			}
		}
	}
	
	
	
	//写入电机PWM
	pwm.SetPwm(motorPWM);
	
}

void FTC_Motor::getPWM(int16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
	*(pwm+4) = motorPWM[4];
	*(pwm+5) = motorPWM[5];	
}

void FTC_Motor::ResetPWM(void)
{
	for(u8 i=0; i< MAXMOTORS ; i++)
		motorPWM[i] = 1000;
}

/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/
