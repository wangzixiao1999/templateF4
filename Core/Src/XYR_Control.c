#include "XYR_Control.h"

float Tim4Rev_freq = 0.f;
volatile bool moveFlag[3] = {true, true, true};
volatile float VOFA_Speed[3] = {20, 20, 200};
volatile float VOFA_Pos[3] = {10, 10, 100};
volatile uint8_t xyr_request = 0; // 非阻塞请求标志，由UART中断置位，在主循环或定时器中处理

XYR_MoveController XYR_MoveCtrl[3] = {
	{XYRMOVE_IDLE, 0, 0, 0}, // X轴
	{XYRMOVE_IDLE, 0, 0, 0}, // Y轴
	{XYRMOVE_IDLE, 0, 0, 0}	 // R轴
};

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
	// if (moveFlag[addr - 1])
	// {
	// 	moveFlag[addr - 1] = false;
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
		// }

		// moveFlag[addr - 1] = true;
	}
}

/**
 * @brief    张大头定长移动
 * @param    addr  	：电机地址
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - 240mm/s
 * @param    position：位置(mm)							，范围0 - 118mm
 */
void XYR_ZDT_Fixed_Length_Move(uint8_t addr, uint8_t dir, float velocity, float position)
{
	if (moveFlag[addr - 1])
	{
		moveFlag[addr - 1] = false;

		if (velocity < 0.f)
			velocity = 0.f;
		else if (velocity > 240.f)
			velocity = 240.f;

		if (position < 0.f)
			position = 0.f;
		else if (position > 118.f)
			position = 118.f;

		if (addr == 1 || addr == 2)
		{
			if (ZDT_state[addr - 1] == 0x03)
			{
				ZDT_X42_V2_Bypass_Position_LV_Control(addr, dir, velocity * 15, position * 72, 0, 0);
				HAL_Delay(10);
				while (!(ZDT_state[addr - 1] == 0x03))
				{
					// HAL_Delay(1);
				}
				ZDT_state[addr - 1] &= ~(0x02);
			}
		}
		moveFlag[addr - 1] = true;
	}
}

/**
 * @brief    转台旋转固定角度
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - mm/s
 * @param    position：位置(mm)							，范围0 - mm
 */
void XYR_HT_Fixed_Length_Move(uint8_t dir, uint32_t velocity, int32_t position)
{
	if (moveFlag[2])
	{
		moveFlag[2] = false;
		HT_DM_S_7010_Set_Position_Max_Speed(1, velocity);
		HAL_Delay(10);
		if (dir)
			position = -position;
		int32_t position_before = HT_Multi_circle_absolute_angle;
		HT_DM_S_7010_Relative_Position_Control(1, position);
		HAL_Delay(10);
		while (!((labs(HT_Multi_circle_absolute_angle - position_before - position) <= 10) && (!HT_speed) && (labs(HT_current) < 200)))
		{
			// HAL_Delay(1);
		}

		moveFlag[2] = true;
	}
}

/**
 * @brief    转台固定速度旋转
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - 240mm/s
 */
void XYR_HT_Fixed_Speed_Move(uint8_t dir, int32_t speed)
{
	if (moveFlag[2])
	{
		moveFlag[2] = false;
		if (dir)
			speed = -speed;
		HT_DM_S_7010_Velocity_Control(1, speed);

		moveFlag[2] = true;
	}
}

/**
 * @brief    XYR解除故障
 */
void XYR_Relieve_Malfunction()
{
	ZDT_X42_V2_Reset_Clog_Pro(0);
	HT_DM_S_7010_Clear_Fault(1);
}

/**
 * @brief    XYR停止移动,R去使能
 */
void XYR_Stop_Move()
{
	ZDT_X42_V2_Stop_Now(0, 0);
	HT_DM_S_7010_Disable_Motor(1);
	moveFlag[2] = true;
}

/**
 * @brief    张大头速度设置(针对VOFA+)
 * @param    addr  	：电机地址
 * @param    velocity：最大速度(mm/s)					，范围0 - 240mm/s
 */
void XYR_ZDT_Speed_Setting_VOFA(uint8_t addr, float velocity)
{
	VOFA_Speed[addr - 1] = velocity;
}

/**
 * @brief    转台速度设置(针对VOFA+)
 * @param    velocity：最大速度(mm/s)				，范围0 - mm/s
 */
void XYR_HT_Speed_Setting_VOFA(uint8_t addr, float velocity)
{
	VOFA_Speed[2] = velocity;
}

/**
 * @brief    张大头位移距离设置(针对VOFA+)
 * @param    addr  	：电机地址
 * @param    position：位移距离(mm)					，范围0 - 240mm
 */
void XYR_ZDT_Pos_Setting_VOFA(uint8_t addr, int32_t position)
{
	VOFA_Pos[addr - 1] = position;
}

/**
 * @brief    转台旋转角度设置(针对VOFA+)
 * @param    position：旋转角度(°)						，范围0 - °
 */
void XYR_HT_Pos_Setting_VOFA(uint8_t addr, int32_t position)
{
	VOFA_Pos[2] = position;
}

/**
 * @brief    小端排序组合返回数值(针对VOFA+)
 * @param    position：旋转角度(°)						，范围0 - °/s
 */
float Parse_Float_LittleEndian(uint8_t *bytes)
{
	uint32_t int_val = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
	return *(float *)&int_val;
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
	static int send_sount = 0;
	if (++send_sount >= 200)
	{
		float values[3];
		values[0] = (float)ZDT_angle[0] / -90.0f;
		values[1] = (float)ZDT_angle[1] / -90.0f;
		values[2] = (float)HT_Single_circle_absolute_angle * 360.f / 16384.f;

		// 发送原始数据（VOFA+使用JustFloat协议）
		HAL_UART_Transmit(&huart1, (uint8_t *)values, sizeof(values), 100);

		// 可选：添加帧尾0x00 0x00 0x80 0x7F（FireWater协议）
		uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
		HAL_UART_Transmit(&huart1, tail, 4, 100);
		// 张大头1号机相电流
		send_sount = 0;
	}
}

/**
 * @brief:串口解析命令
 */
void XYR_Read_USB_Commend()
{
	float value = 0.f;
	switch (xyr_request)
	{
	case 0xA0:
		xyr_request = 0;
		XYR_Collision_Home(0);
		break;
	case 0xA1:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		XYR_ZDT_Speed_Setting_VOFA(1, value);
		break;
	case 0xA2:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		XYR_ZDT_Pos_Setting_VOFA(1, value / 1000.0f);
		break;
	case 0xA3:
		xyr_request = 0;
		XYR_ZDT_Fixed_Length_Move(1, 1, VOFA_Speed[0], VOFA_Pos[0]);
		break;
	case 0xA4:
		xyr_request = 0;
		XYR_ZDT_Fixed_Length_Move(1, 0, VOFA_Speed[0], VOFA_Pos[0]);
		break;
	default:
		break;
	}
}

/**
 * @brief:电机1状态机
 */
void XYR_Update_Axis_StateMachine(XYR_MoveController *XYR_MoveCtrl)
{
	uint32_t current_time = HAL_GetTick();
	switch (XYR_MoveCtrl->state)
	{
        case XYRMOVE_IDLE:
            break;

        case XYRMOVE_START_MOVE:
            // 等待逻辑

            // 检查回零完成条件...
            break;

        case XYRMOVE_WAIT_STOP:
            // 运动控制逻辑


            break;

        case XYRMOVE_COMPLETE:
            // 完成后的处理

            break;
	}
}
