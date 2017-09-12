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
	rc.jyszz=1;
	rc.Is_Fly_Over=0;
	rc.First_Back=1;
	rc.First_TakeOff=1;
	
	//��ʼ���ɿذ��Ӳ������
	FTC_Hexacopter_board_Init();
	
	//��ʼ������
	param.Init();
	
	//��ʼ��IMU�����Բ�����Ԫ��
	imu.Init();	
	
	nav.Init();
	
	while(1)
	{
		FTC_Loop();
	}
	
	return 0;
}

/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/
