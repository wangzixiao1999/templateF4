#include "XYR_Control.h"

float Tim4Rev_freq = 0.f;
int32_t cur = 0;
/**
 * @brief    初始化XYR三轴位移台
 */
void XYR_Init()
{
	// ZDT_X42_V2_Origin_Modify_Params(0, false, 2, 0, 1000, 10000, 500, 150, 150, false);
}

/**
 * @brief    碰撞回零
 * @param    addr  ：电机地址
 */
void XYR_Collision_Home(uint8_t addr)
{
	ZDT_X42_V2_Origin_Trigger_Return(addr, 2, false);
	HAL_Delay(10);
	while (can.rxData[0] != 0x9A || can.rxData[1] != 0x9F)
	{
		can.rxFrameFlag = false;
	}
	ZDT_X42_V2_Bypass_Position_LV_Control(addr, 1, 500, 4500, 0, 0);
	HAL_Delay(10);
	while (can.rxData[0] != 0xFB || can.rxData[1] != 0x9F)
	{
		can.rxFrameFlag = false;
	}
}

/**
 * @brief:中断函数使用
 */
void Controller_Update_Callback(void)
{
	static volatile uint32_t preTick_Tim4 = 0;
	static uint8_t HT_request_index = 0;
	//static uint8_t ZDT_request_index = 0;
	switch (HT_request_index)
	{
	case 0:
		HT_DM_S_7010_Read_Current_Q_Axis(1); // 读取转台相电流
	case 1:
		HT_DM_S_7010_Read_Speed(1); // 读取转台速度
	case 2:
		HT_DM_S_7010_Read_Absolute_Angle(1); // 读取转台角度
	case 3:
		ZDT_X42_V2_Read_Sys_Params(1, S_State); // 读取张大头1号机相电流
	case 4:
		ZDT_X42_V2_Read_Sys_Params(2, S_State); // 读取张大头2号机相电流
	}

	// switch (ZDT_request_index)
	// {

	// }

	HT_request_index = (HT_request_index + 1) % 5;
	// ZDT_request_index = (ZDT_request_index + 1) % 2;

	uint32_t currTick_Tim4 = HAL_GetTick();
	Tim4Rev_freq = 1000.f / (currTick_Tim4 - preTick_Tim4);
	preTick_Tim4 = currTick_Tim4;
	// 每50ms发送一次数据
	//  static int send_sount=0;
	//  if(++send_sount>=5)
	//  {

	// }
}