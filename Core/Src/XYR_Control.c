#include "XYR_Control.h"

/**
  * @brief    碰撞回零
  * @param    addr  ：电机地址
  */
void XYR_Collision_Home(uint8_t addr)
{
	ZDT_X42_V2_Origin_Trigger_Return(addr, 2,   false);
	HAL_Delay(10);
	while(can.rxData[0] != 0x9A || can.rxData[1] != 0x9F)
	{
    	can.rxFrameFlag = false;
    }
		HAL_Delay(500);
	ZDT_X42_V2_Bypass_Position_LV_Control( addr, 1, 500, 4500, 0, 0);
	HAL_Delay(10);
	while(can.rxData[0] != 0xFB || can.rxData[1] != 0x9F)
	{
    	can.rxFrameFlag = false;
    }
}