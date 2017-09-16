/******************** (C) COPYRIGHT 2015 FTC ***************************
 * ����		 ��FTC
 * �ļ���  ��FTC_Motor.cpp
 * ����    �����������غ���
**********************************************************************************/
#include "FTC_Motor.h"

FTC_Motor motor;

void FTC_Motor::writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	//����X��
	motorPWM[2] = throttle - 0.5 * pidTermRoll + 0.866 *  pidTermPitch + pidTermYaw; //����
	motorPWM[1] = throttle - 0.5 * pidTermRoll - 0.866 *  pidTermPitch + pidTermYaw; //ǰ��
	motorPWM[0] = throttle + 0.5 * pidTermRoll + 0.866 *  pidTermPitch - pidTermYaw; //����
	motorPWM[3] = throttle + 0.5 * pidTermRoll - 0.866 *  pidTermPitch - pidTermYaw; //ǰ��
	motorPWM[5] = throttle - pidTermRoll - pidTermYaw;	//��
	motorPWM[4] = throttle + pidTermRoll + pidTermYaw;	//��
	
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
		//���Ƶ��PWM����С�����ֵ
		motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}

	//���δ����,�򽫵���������Ϊ���
	
	if(!ftc.f.ARMED)	
		ResetPWM();

	if(!ftc.f.ALTHOLD && rc.rawData[THROTTLE] < RC_MINCHECK)
		ResetPWM();

	
	//�׷�.Version1
	
	if (ftc.f.ARMED) {
		if (imu.Acc.z > 2*ACC_1G || flag == 0) {
			if (time < 3000) {
				rc.jyszz = 0;
				
				uint16_t throttle_2 = 1800;
				
				if (time >= 1000 && time <3000) 
					throttle_2 = throttle_2 - 30 * (time / 500 - 2);
				
				motorPWM[2] = throttle_2 - 0.5 * pidTermRoll + 0.866 *  pidTermPitch + pidTermYaw; //����
				motorPWM[1] = throttle_2 - 0.5 * pidTermRoll - 0.866 *  pidTermPitch + pidTermYaw; //ǰ��
				motorPWM[0] = throttle_2 + 0.5 * pidTermRoll + 0.866 *  pidTermPitch - pidTermYaw; //����
				motorPWM[3] = throttle_2 + 0.5 * pidTermRoll - 0.866 *  pidTermPitch - pidTermYaw; //ǰ��
				motorPWM[5] = throttle_2 - pidTermRoll - pidTermYaw;	//��
				motorPWM[4] = throttle_2 + pidTermRoll + pidTermYaw;	//��
				time++;
				flag = 0;
			}
			
			if (time >= 3000) {
				flag = 1;
				time = 0;
			}
		}
	}
	
	
	
	//д����PWM
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
