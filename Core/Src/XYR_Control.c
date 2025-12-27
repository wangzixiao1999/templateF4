#include "XYR_Control.h"

float Tim4Rev_freq = 0.f;
volatile bool moveFlag = true;
/**
 * @brief    初始化XYR三轴位移台
 */
void XYR_Init()
{

}

/**
 * @brief    碰撞回零
 * @param    addr  ：电机地址
 */
void XYR_Collision_Home(uint8_t addr)
{
	if (moveFlag)
	{
		moveFlag = false;
	if (addr == 1 || addr == 2)
	{
		ZDT_X42_V2_Origin_Trigger_Return(addr, 2, false);
		HAL_Delay(10);
		while (!(ZDT_state[addr - 1] == 0x03))
		{
			// HAL_Delay(1);
		}

		ZDT_X42_V2_Bypass_Position_LV_Control(addr, 1, 500, 4500, 0, 0);
		HAL_Delay(10);
		while (!(ZDT_state[addr - 1] == 0x03))
		{
			// HAL_Delay(1);
		}
	}
	else if (addr == 0)
	{
		ZDT_X42_V2_Origin_Trigger_Return(addr, 2, false);
		HAL_Delay(10);
		while (!((ZDT_state[0] == 0x03) && (ZDT_state[1] == 0x03)))
		{
			// HAL_Delay(1);
		}
		ZDT_X42_V2_Bypass_Position_LV_Control(addr, 1, 500, 4500, 0, 0);
		HAL_Delay(10);
		while (!((ZDT_state[0] == 0x03) && (ZDT_state[1] == 0x03)))
		{
			// HAL_Delay(1);
		}
	}

	moveFlag = true;
	}
}

/**
 * @brief    张大头定长移动
 * @param    addr  	：电机地址
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - 240mm/s
 * @param    position：位置(mm)							，范围0 - 100mm
 */
void XYR_Fixed_Length_Move(uint8_t addr, uint8_t dir, float velocity, float position)
{
	if (moveFlag)
	{
		moveFlag = false;

		if (velocity < 0.f)
			velocity = 0.f;
		else if (velocity > 240.f)
			velocity = 240.f;

		if (position < 0.f)
			position = 0.f;
		else if (position > 100.f)
			position = 100.f;

		if (addr == 1 || addr == 2)
		{
	if (ZDT_state[addr - 1] == 0x03)
	{
		ZDT_X42_V2_Bypass_Position_LV_Control(addr, dir, velocity * 15, position * 90, 0, 0);
		HAL_Delay(10);
		while (!(ZDT_state[addr - 1] == 0x03))
		{
			// HAL_Delay(1);
		}
		ZDT_state[addr - 1] &= ~(0x02);
	}
		}
		moveFlag = true;
	}
}

/**
 * @brief:指令发送端
 */
void Controller_Update_Callback(void)
{
	static volatile uint32_t preTick_Tim4 = 0;
	static uint8_t HT_request_index = 0;
	static uint8_t ZDT_request_index = 0;
	switch (HT_request_index)
	{
	case 0:
		HT_DM_S_7010_Read_Current_Q_Axis(1); // 读取转台相电流

	case 1:
		HT_DM_S_7010_Read_Speed(1); // 读取转台速度

	case 2:
		HT_DM_S_7010_Read_Absolute_Angle(1); // 读取转台角度
	}

	switch (ZDT_request_index)
	{
	case 0:
		ZDT_X42_V2_Read_Sys_Params(1, S_State); // 读取张大头1号机相电流
	case 1:
		ZDT_X42_V2_Read_Sys_Params(2, S_State); // 读取张大头2号机相电流
	case 2:
		ZDT_cmdSend(); // 发送装载好的命令
	}

	HT_request_index = (HT_request_index + 1) % 3;
	ZDT_request_index = (ZDT_request_index + 1) % 3;

	uint32_t currTick_Tim4 = HAL_GetTick();
	Tim4Rev_freq = 1000.f / (currTick_Tim4 - preTick_Tim4);
	preTick_Tim4 = currTick_Tim4;
	// 每50ms发送一次数据
	//  static int send_sount=0;
	//  if(++send_sount>=5)
	//  {

	// }
}