/******************** (C) COPYRIGHT 2015 FTC ***************************
 * ����		 ��FTC
 * �ļ���  ��FtcCopter.cpp
 * ����    ��Filter����̽����V1.0΢�����������
 * ����汾��V1.0
 * ʱ��		 ��2015/12/1
**********************************************************************************/
#include "FTC_Config.h"

int main(void)
{
	//��ʼ���ɿذ��Ӳ������
	FTC_Hexacopter_board_Init();
	
	//��ʼ������
	param.Init();
	
	//��ʼ��IMU�����Բ�����Ԫ��
	imu.Init();	
	
	nav.Init();
	led.Init();
	while(1)
	{
		FTC_Loop();
		led.ON1();
		if(rc.rawData[AUX2]>1400 && rc.rawData[AUX2]<1600)
		{
			led.OFF1();
		}
	}
	
	return 0;
}

/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/
