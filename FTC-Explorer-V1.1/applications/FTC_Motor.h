#ifndef __FTC_MOTOR_H
#define __FTC_MOTOR_H

#include "FTC_Config.h"

#define MINTHROTTLE 1100
#define MAXTHROTTLE 1900

class FTC_Motor
{

public:
	
	void writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw);
	
	void getPWM(int16_t* pwm);

private:
	
	int16_t motorPWM[6];	

	int16_t time;//抛飞功能，记录抛飞时间
	int16_t flag;

	void ResetPWM(void);

};

extern FTC_Motor motor;

#endif





