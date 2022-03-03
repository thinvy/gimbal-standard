#include "DebugThread.h"
#include "cmsis_os.h"

#include "AttitudeThread.h"

uint16_t tx[3];
extern fp32 INS_angle[3];
//暂时用来玩匿名上位机
void DebugThread(void const * pvParameters)
{
	osDelay(500);
	while(1)
	{
//		for(uint8_t i = 0;i<3;i++) tx[i] = (uint16_t)(INS_angle[i]/ 3.14* 180);
//		Test_Send_User(tx[0], tx[1], tx[2]);
		osDelay(10);
	}
}


