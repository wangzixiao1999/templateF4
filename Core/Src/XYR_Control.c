#include "XYR_Control.h"

float Tim4Rev_freq = 0.f;
volatile bool moveFlag[3] = {true, true, true};
// 非阻塞状态机相关
typedef enum
{
	XYR_IDLE = 0,
	XYR_START,
	XYR_WAIT_ORIGIN,
	XYR_TRIGGER_RETURN_SENT,
	XYR_WAIT_POSITION_DONE,
	XYR_RUNNING_SPEED,
	XYR_DONE
} XYR_State_t;

typedef struct
{
	XYR_State_t state;
	// 通用参数
	uint8_t addr; // 1 or 2 for ZDT, 3 for HT (we'll index HT as 3 internally)
	uint8_t dir;
	float velocity;
	float position_f;
	int32_t position_i;
	uint32_t velocity_u32;
	int32_t speed;
	// HT specific
	int32_t ht_position_before;
	int32_t ht_target_rel;
} XYR_SM_t;

static XYR_SM_t xyr_sm[3]; // 0->ZDT addr1, 1->ZDT addr2, 2->HT

// 内部：开始并由状态机推进
static void XYR_SM_Start_ZDT_Home(uint8_t idx)
{
	xyr_sm[idx].state = XYR_START;
	xyr_sm[idx].addr = idx + 1;
	moveFlag[idx] = false; // 标记为忙
}

static void XYR_SM_Start_ZDT_Move(uint8_t idx, uint8_t dir, float velocity, float position)
{
	xyr_sm[idx].state = XYR_START;
	xyr_sm[idx].addr = idx + 1;
	xyr_sm[idx].dir = dir;
	xyr_sm[idx].velocity = velocity;
	xyr_sm[idx].position_f = position;
	moveFlag[idx] = false;
}

static void XYR_SM_Start_HT_RelPos(int32_t position, uint32_t velocity)
{
	xyr_sm[2].state = XYR_START;
	xyr_sm[2].addr = 3; // HT
	xyr_sm[2].position_i = position;
	xyr_sm[2].velocity_u32 = velocity;
	moveFlag[2] = false;
}

static void XYR_SM_Start_HT_Speed(int32_t speed)
{
	xyr_sm[2].state = XYR_RUNNING_SPEED;
	xyr_sm[2].addr = 3;
	xyr_sm[2].speed = speed;
	moveFlag[2] = false;
}

// 状态机推进函数，需要被周期性调用（如Controller_Update_Callback内）
static void XYR_SM_Update(void)
{
	// ZDT axis 1 and 2
	for (uint8_t i = 0; i < 2; ++i)
	{
		switch (xyr_sm[i].state)
		{
		case XYR_IDLE:
		case XYR_DONE:
			break;
		case XYR_START:
			// 发送回零触发
			ZDT_X42_V2_Origin_Trigger_Return(xyr_sm[i].addr, 2, false);
			xyr_sm[i].state = XYR_WAIT_ORIGIN;
			break;
		case XYR_WAIT_ORIGIN:
			if (ZDT_state[xyr_sm[i].addr - 1] == 0x03)
			{
				// 发送位置指令（如果是home流程，走到位置0附近）
				ZDT_X42_V2_Bypass_Position_LV_Control(xyr_sm[i].addr, 1, 500, 4500, 0, 0);
				xyr_sm[i].state = XYR_WAIT_POSITION_DONE;
			}
			break;
		case XYR_WAIT_POSITION_DONE:
			if (ZDT_state[xyr_sm[i].addr - 1] == 0x03)
			{
				xyr_sm[i].state = XYR_DONE;
				moveFlag[i] = true;
			}
			break;
		default:
			break;
		}
	}

	// HT axis (index 2)
	switch (xyr_sm[2].state)
	{
	case XYR_IDLE:
	case XYR_DONE:
		break;
	case XYR_START:
		// 设置速度上限，然后发出相对位置指令
		HT_DM_S_7010_Set_Position_Max_Speed(1, xyr_sm[2].velocity_u32);
		// 记录当前多圈角度作为基准
		xyr_sm[2].ht_position_before = HT_Multi_circle_absolute_angle;
		HT_DM_S_7010_Relative_Position_Control(1, xyr_sm[2].position_i);
		xyr_sm[2].state = XYR_WAIT_POSITION_DONE;
		break;
	case XYR_WAIT_POSITION_DONE:
		// 完成条件尽量复用原有逻辑：角度接近目标、速度为0、电流正常
		if ((labs(HT_Multi_circle_absolute_angle - xyr_sm[2].ht_position_before - xyr_sm[2].position_i) <= 10) && (!HT_speed) && (labs(HT_current) < 200))
		{
			xyr_sm[2].state = XYR_DONE;
			moveFlag[2] = true;
		}
		break;
	case XYR_RUNNING_SPEED:
		// 直接控制速度即可，保持状态直到外部停止或再次调用停止
		HT_DM_S_7010_Velocity_Control(1, xyr_sm[2].speed);
		// remain in RUNNING until user calls Stop or another command overwrites state
		break;
	default:
		break;
	}
}
/**
 * @brief    初始化XYR三轴位移台
 */
void XYR_Init()
{
	// 初始化状态机为idle
	for (int i = 0; i < 3; ++i)
	{
		xyr_sm[i].state = XYR_IDLE;
		moveFlag[i] = true;
	}
}

/**
 * @brief    碰撞回零
 * @param    addr  ：电机地址
 */
void XYR_Collision_Home(uint8_t addr)
{
	// 非阻塞：仅启动状态机，由周期回调推进
	if (addr == 0)
	{
		// 同步启动两个ZDT轴的回零
		for (uint8_t i = 0; i < 2; ++i)
		{
			if (moveFlag[i])
				XYR_SM_Start_ZDT_Home(i);
		}
	}
	else if ((addr == 1) || (addr == 2))
	{
		uint8_t idx = addr - 1;
		if (moveFlag[idx])
			XYR_SM_Start_ZDT_Home(idx);
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
	// 非阻塞：只做参数校验并启动状态机，由XYR_SM_Update推进完成
	if ((addr == 1) || (addr == 2))
	{
		uint8_t idx = addr - 1;
		if (!moveFlag[idx])
			return; // 忙碌中

		if (velocity < 0.f)
			velocity = 0.f;
		else if (velocity > 240.f)
			velocity = 240.f;

		if (position < 0.f)
			position = 0.f;
		else if (position > 118.f)
			position = 118.f;

		XYR_SM_Start_ZDT_Move(idx, dir, velocity, position);
		// 状态机会在XYR_SM_Update里发送指令，当ZDT_ready后会发出相应位置命令
	}
}

/**
 * @brief    转台旋转固定角度
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - 240mm/s
 * @param    position：位置(mm)							，范围0 - 118mm
 */
void XYR_HT_Fixed_Length_Move(uint8_t dir, uint32_t velocity, int32_t position)
{
	if (!moveFlag[2])
		return; // 忙碌中

	if (dir)
		position = -position;

	XYR_SM_Start_HT_RelPos(position, velocity);
}

/**
 * @brief    转台固定速度旋转
 * @param    dir     ：方向								，0为归零方向，其余值为电机方向
 * @param    velocity：最大速度(mm/s)					，范围0 - 240mm/s
 */
void XYR_HT_Fixed_Speed_Move(uint8_t dir, int32_t speed)
{
	// 非阻塞速度控制：直接设置并进入RUNNING状态
	if (!moveFlag[2])
		return; // 忙碌中

	if (dir)
		speed = -speed;

	XYR_SM_Start_HT_Speed(speed);
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
}

/**
 * @brief:指令发送端
 */
void Controller_Update_Callback(void)
{
	// 推进XYR状态机（非阻塞动作由此推进）
	XYR_SM_Update();
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