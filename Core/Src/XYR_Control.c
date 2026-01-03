#include "XYR_Control.h"

float Tim4Rev_freq = 0.f;
volatile bool moveFlag[3] = {true, true, true};

volatile float VOFA_Speed[3] = {20, 20, 20};
volatile float VOFA_Pos[3] = {10, 10, 0};
volatile uint8_t VOFA_Scan[2] = {10, 10};
volatile bool AutoScanFlag = true;
volatile bool USB_Send_Flag = false;

volatile uint8_t xyr_request = 0;

XYR_MotorStateSpace XYR_MotorState[3] = {0};
AutoScanContext g_auto_scan = {0};
/**
 * @brief    初始化XYR三轴位移台
 */
void XYR_Init()
{
	for (int i = 0; i < MAX_MOTORS; i++)
	{
		XYR_MotorState[i].state = XYRMOVE_IDLE;
		XYR_MotorState[i].axis = i;
		XYR_MotorState[i].dir = 0;
		XYR_MotorState[i].velocity = 0;
		XYR_MotorState[i].position = 0;
		XYR_MotorState[i].start_time = 0;
		XYR_MotorState[i].elapsed_time = 0;
		XYR_MotorState[i].move_flag = false;
		XYR_MotorState[i].command_sent = false;
	}
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
		XYR_MotorState_Transition(addr, XYRMOVE_WAIT_COMMEND);
		ZDT_X42_V2_Origin_Trigger_Return(addr, 2, false);
	}
	else if (addr == 0)
	{
		XYR_MotorState_Transition(1, XYRMOVE_WAIT_COMMEND);
		XYR_MotorState_Transition(2, XYRMOVE_WAIT_COMMEND);
		ZDT_X42_V2_Origin_Trigger_Return(1, 2, false);
		ZDT_X42_V2_Origin_Trigger_Return(2, 2, false);
	}
}

/**
 * @brief    张大头定长移动
 * @param    addr  	：电机地址
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - 250mm/s
 * @param    position：位置(mm)							，范围0 - 118mm
 */
void XYR_ZDT_Fixed_Length_Move(uint8_t addr, uint8_t dir, float velocity, float position)
{
	if (addr == 0 || addr > MAX_MOTORS - 1)
	{
		return; // 错误
	}

	if (XYR_MotorState[addr - 1].state != XYRMOVE_IDLE)
	{
		return;
	}

	if (velocity < 0.f)
		velocity = 0.f;
	else if (velocity > 250.f)
		velocity = 250.f;

	if (position < 0.f)
		position = 0.f;
	else if (position > 118.f)
		position = 118.f;

	XYR_MotorState_Transition(addr, XYRMOVE_WAIT_COMMEND);
	ZDT_X42_V2_Bypass_Position_LV_Control(addr, dir, velocity * 12, position * 72, 0, 0);
}

/**
 * @brief    转台旋转固定角度
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - mm/s
 * @param    position：位置(mm)							，范围0 - mm
 */
void XYR_HT_Fixed_Length_Move(uint8_t dir, uint32_t velocity, int32_t position)
{
	HT_DM_S_7010_Set_Position_Max_Speed(3, velocity * 100);
	HAL_Delay(10);
	if (dir)
		position = -position;
	int32_t position_before = HT_Multi_circle_absolute_angle;
	HT_DM_S_7010_Relative_Position_Control(3, position);
	HAL_Delay(10);
	// while (!((labs(HT_Multi_circle_absolute_angle - position_before - position) <= 10) && (!HT_speed) && (labs(HT_current) < 200)))
	// {
	// 	// HAL_Delay(1);
	// }
}

/**
 * @brief    转台固定速度旋转
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - 300Rpm
 */
void XYR_HT_Fixed_Speed_Move(uint8_t dir, int32_t velocity)
{
	if (dir)
		velocity = -velocity;
	HT_DM_S_7010_Velocity_Control(3, velocity * 100);
}

/**
 * @brief    XYR解除故障
 */
void XYR_Relieve_Malfunction()
{
	ZDT_X42_V2_Reset_Clog_Pro(0);
	HT_DM_S_7010_Clear_Fault(3);
}

/**
 * @brief    XYR停止移动,R去使能
 */
void XYR_Stop_Move()
{
	ZDT_X42_V2_Stop_Now(0, 0);
	HT_DM_S_7010_Disable_Motor(3);
	for (int i = 0; i < 2; i++)
		moveFlag[i] = true;
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
void XYR_ZDT_Pos_Setting_VOFA(uint8_t addr, float position)
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
 * @brief    开始面扫(针对VOFA+)
 * @param    x 行数
 * @param    y 列数
 */
void XYR_AutoScan_Start(uint8_t x, uint8_t y)
{
	AutoScanFlag = true;
	if (x == 0 || y == 0)
	{
		return; // 无效参数
	}

	if ((XYR_MotorState[0].state == XYRMOVE_IDLE) && (XYR_MotorState[1].state == XYRMOVE_IDLE))
	{
		// 初始化面扫上下文
		g_auto_scan.state = SCAN_RUNNING;
		g_auto_scan.current_row = 0;
		g_auto_scan.current_col = 0;
		g_auto_scan.total_rows = x;
		g_auto_scan.total_cols = y;
		g_auto_scan.row_direction = false;
		g_auto_scan.last_update_time = HAL_GetTick();
		g_auto_scan.waiting_for_move_complete = false;

		AutoScanFlag = true;

		if (g_auto_scan.row_direction)
		{
			XYR_ZDT_Fixed_Length_Move(2, 1, VOFA_Speed[1], VOFA_Pos[1]);
		}
		else
		{
			XYR_ZDT_Fixed_Length_Move(2, 0, VOFA_Speed[1], VOFA_Pos[1]);
		}

		g_auto_scan.waiting_for_move_complete = true;
	}
}

/**
 * @brief    停止面扫(针对VOFA+)
 */
void XYR_AutoScan_Stop(void)
{
	g_auto_scan.state = SCAN_IDLE;
	AutoScanFlag = false;
	// 停止所有运动
	XYR_Stop_Move();
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
		HT_DM_S_7010_Read_Current_Q_Axis(3); // 读取转台相电流

		break;
	case 1:
		HT_DM_S_7010_Read_Speed(3); // 读取转台速度

		break;
	case 2:
		HT_DM_S_7010_Read_Absolute_Angle(3); // 读取转台角度
		break;
	}

	switch (ZDT_request_index)
	{
	case 0:
		ZDT_X42_V2_Read_Sys_Params(1, S_State); // 读取张大头1号机
		break;
	case 1:
		ZDT_cmdSend(1); // 发送装载好的张大头1号机的命令
		break;
	case 2:
		ZDT_X42_V2_Read_Sys_Params(2, S_State); // 读取张大头2号机
		break;
	case 3:
		ZDT_cmdSend(2); // 发送装载好的张大头2号机的命令
		break;
	}

	HT_request_index = (HT_request_index + 1) % 3;
	ZDT_request_index = (ZDT_request_index + 1) % 4;

	uint32_t currTick_Tim4 = HAL_GetTick();
	Tim4Rev_freq = 1000.f / (currTick_Tim4 - preTick_Tim4);
	preTick_Tim4 = currTick_Tim4;
	// 每50ms发送一次数据(改：在main里面)
	static int send_sount = 0;
	if (++send_sount >= 50)
	{
		USB_Send_Flag = true;
		send_sount = 0;
	}
}

/**
 * @brief:串口发送状态更新
 */
void XYR_Send_USB_Commend()
{
	float values[3];
	values[0] = (float)ZDT_angle[0];
	values[1] = (float)ZDT_angle[1];
	values[2] = (float)HT_Single_circle_absolute_angle * 360.f / 16384.f;

	// 发送原始数据（VOFA+使用JustFloat协议）
	HAL_UART_Transmit(&huart1, (uint8_t *)values, sizeof(values), 100);

	// 可选：添加帧尾0x00 0x00 0x80 0x7F（FireWater协议）
	uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
	HAL_UART_Transmit(&huart1, tail, 4, 100);
	// 张大头1号机相电流
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
		XYR_ZDT_Fixed_Length_Move(1, 0, VOFA_Speed[0], VOFA_Pos[0]);
		break;
	case 0xA4:
		xyr_request = 0;
		XYR_ZDT_Fixed_Length_Move(1, 1, VOFA_Speed[0], VOFA_Pos[0]);
		break;
	case 0xB1:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		XYR_ZDT_Speed_Setting_VOFA(2, value);
		break;
	case 0xB2:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		XYR_ZDT_Pos_Setting_VOFA(2, value / 1000.0f);
		break;
	case 0xB3:
		xyr_request = 0;
		XYR_ZDT_Fixed_Length_Move(2, 0, VOFA_Speed[1], VOFA_Pos[1]);
		break;
	case 0xB4:
		xyr_request = 0;
		XYR_ZDT_Fixed_Length_Move(2, 1, VOFA_Speed[1], VOFA_Pos[1]);
		break;
	case 0xC1:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		XYR_HT_Speed_Setting_VOFA(3, value);
		break;
	case 0xC2:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		XYR_HT_Pos_Setting_VOFA(3, value / 360.f * 16384.f);
		break;
	case 0xC3:
		xyr_request = 0;
		if (!VOFA_Pos[2])
			XYR_HT_Fixed_Speed_Move(1, VOFA_Speed[2]);
		else
			XYR_HT_Fixed_Length_Move(1, VOFA_Speed[2], VOFA_Pos[2]);
		break;
	case 0xC4:
		xyr_request = 0;
		if (!VOFA_Pos[2])
			XYR_HT_Fixed_Speed_Move(0, VOFA_Speed[2]);
		else
			XYR_HT_Fixed_Length_Move(0, VOFA_Speed[2], VOFA_Pos[2]);
		break;
	case 0xD0:
		xyr_request = 0;
		XYR_Stop_Move();
		break;
	case 0xD1:
		xyr_request = 0;
		XYR_Relieve_Malfunction();
		break;
		// 面扫管理
	case 0xE1:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		VOFA_Scan[0] = (uint8_t)value;
		break;
	case 0xE2:
		xyr_request = 0;
		value = Parse_Float_LittleEndian(&process_buffer[1]);
		VOFA_Scan[1] = (uint8_t)value;
		break;
	case 0xE3:
		xyr_request = 0;
		XYR_AutoScan_Start(VOFA_Scan[0], VOFA_Scan[1]);
		break;
	case 0xE4:
		xyr_request = 0;
		XYR_AutoScan_Stop();
		break;
	default:
		break;
	}
}

/**
 * @brief:状态转换
 */
void XYR_MotorState_Transition(uint8_t addr, XYR_State new_state)
{
	if (addr == 0 || addr > MAX_MOTORS)
	{
		return; // 错误
	}
	XYR_MotorState[addr - 1].state = new_state;
	switch (new_state)
	{
	case XYRMOVE_IDLE:
		XYR_MotorState[addr - 1].command_sent = false;
		XYR_MotorState[addr - 1].move_flag = false;
		break;

	case XYRMOVE_WAIT_COMMEND:

		break;

	case XYRMOVE_COMMEND_LOADING:

		break;

	case XYRMOVE_WAIT_STOP:
		XYR_MotorState[addr - 1].command_sent = true;
		XYR_MotorState[addr - 1].move_flag = true;
		break;

	case XYRMOVE_COMPLETE:
		XYR_MotorState[addr - 1].move_flag = false;

		break;

	default:
		break;
	}
}

/**
 * @brief:更新状态空间
 */
void XYR_MotorState_Update(uint8_t addr)
{
	if (addr == 0 || addr > MAX_MOTORS)
	{
		return;
	}

	uint8_t index = addr - 1;

	switch (XYR_MotorState[index].state)
	{
	case XYRMOVE_IDLE:
		break;

	case XYRMOVE_WAIT_COMMEND:
	{
		if (XYR_MotorState[index].command_sent == true)
		{
			XYR_MotorState_Transition(addr, XYRMOVE_COMMEND_LOADING);
			XYR_MotorState[index].start_time = HAL_GetTick();
			return;
		}
	}
	break;

	case XYRMOVE_COMMEND_LOADING:
		XYR_MotorState[index].elapsed_time = HAL_GetTick() - XYR_MotorState[index].start_time;
		if (XYR_MotorState[index].elapsed_time >= COMMEND_LOADING_TIME)
		{
			XYR_MotorState_Transition(addr, XYRMOVE_WAIT_STOP);
			return;
		}
		break;

	case XYRMOVE_WAIT_STOP:
		if (ZDT_state[index] & 0x02)
		{
			ZDT_state[index] &= ~(0x02);
			XYR_MotorState_Transition(addr, XYRMOVE_COMPLETE);
			XYR_MotorState[index].start_time = HAL_GetTick();
			return;
		}
		break;
	case XYRMOVE_COMPLETE:
		XYR_MotorState[index].elapsed_time = HAL_GetTick() - XYR_MotorState[index].start_time;
		if (XYR_MotorState[index].elapsed_time >= COMMEND_LOADING_TIME)
		{
			XYR_MotorState_Transition(addr, XYRMOVE_IDLE);
			return;
		}
		break;
	default:
		break;
	}
}

/**
 * @brief    更新面扫状态
 */
void XYR_AutoScan_Update(void)
{
	static uint32_t last_update_time = 0;
	uint32_t current_time = HAL_GetTick();

	// 限制更新频率（例如每10ms更新一次）
	if (current_time - last_update_time < 10)
	{
		return;
	}
	last_update_time = current_time;

	// 如果面扫未运行，直接返回
	if (g_auto_scan.state != SCAN_RUNNING || !AutoScanFlag)
	{
		return;
	}

	// 如果正在等待移动完成
	if (g_auto_scan.waiting_for_move_complete)
	{
		// 检查当前Y轴是否移动完成（假设使用axis 2）
		if (XYR_MotorState[0].state == XYRMOVE_IDLE && XYR_MotorState[1].state == XYRMOVE_IDLE)
		{

			g_auto_scan.waiting_for_move_complete = false;

			// 更新列计数器
			g_auto_scan.current_col++;

			// 如果当前行已完成所有列
			if (g_auto_scan.current_col >= g_auto_scan.total_cols)
			{
				// 移动到下一行
				g_auto_scan.current_row++;
				g_auto_scan.current_col = 0;

				// 如果所有行都已完成
				if (g_auto_scan.current_row >= g_auto_scan.total_rows)
				{
					g_auto_scan.state = SCAN_COMPLETE;
					AutoScanFlag = false;
					return;
				}

				// 换行：X轴移动一个位置
				XYR_ZDT_Fixed_Length_Move(1, 0, VOFA_Speed[0], VOFA_Pos[0]);

				// 切换行方向
				g_auto_scan.row_direction = !g_auto_scan.row_direction;

				// 等待X轴移动完成
				// 这里可以设置一个标志，在下次更新时检查
				// 或者立即开始Y轴移动（取决于你的需求）
				g_auto_scan.waiting_for_move_complete = true;
			}
			else
			{
				// 当前行还有更多列，继续Y轴移动
				if (g_auto_scan.row_direction)
				{
					// 从左到右，Y轴正向移动
					XYR_ZDT_Fixed_Length_Move(2, 1, VOFA_Speed[1], VOFA_Pos[1]);
				}
				else
				{
					// 从右到左，Y轴反向移动
					XYR_ZDT_Fixed_Length_Move(2, 0, VOFA_Speed[1], VOFA_Pos[1]);
				}

				g_auto_scan.waiting_for_move_complete = true;
			}
		}
	}
}