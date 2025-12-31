#ifndef __XYR_CONTROL_H
#define __XYR_CONTROL_H

// XYR_Control.h
// XYR三轴位移台控制

#include "ZDT_X42_V2.h"
#include "HT_DM_S_7010.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usart.h"

#define MAX_MOTORS 3
#define COMMEND_LOADING_TIME 10

// 轴状态定义
typedef enum
{
	XYRMOVE_IDLE,			 // 空闲
	XYRMOVE_WAIT_COMMEND,	 // 等待命令发送
	XYRMOVE_COMMEND_LOADING, // 等待命令装载                                                                                                                                                                   , // 开始运动
	XYRMOVE_WAIT_STOP,		 // 等待完成
	XYRMOVE_COMPLETE,		 // 运动完成
	XYRMOVE_BLOCKAGE		 // 运动限位
} XYR_State;

typedef struct
{
	// 设备状态维度
	XYR_State state; // 当前状态
	uint8_t axis;	 // 轴编号

	// 运动参数维度
	bool dir;		   // 方向
	uint32_t velocity; // 速度
	uint32_t position; // 位置

	// 时间维度
	uint32_t start_time;   // 开始时间
	uint32_t elapsed_time; // 已用时间

	// 执行状态维度
	bool command_sent; // 空闲标志
	bool move_flag;	   // 移动标志
	uint8_t cmd[32];   // 接收命令
	uint8_t cmdLength; // 接收命令长度

} XYR_MotorStateSpace;

extern XYR_MotorStateSpace XYR_MotorState[3];

void XYR_Init();
void XYR_Collision_Home(uint8_t addr);													   // 碰撞回零
void XYR_ZDT_Fixed_Length_Move(uint8_t addr, uint8_t dir, float velocity, float position); // 张大头定长移动
void XYR_HT_Fixed_Length_Move(uint8_t dir, uint32_t velocity, int32_t position);		   // 转台旋转固定角度
void XYR_HT_Fixed_Speed_Move(uint8_t dir, int32_t speed);								   // 转台固定速度旋转
void XYR_Stop_Move();																	   // XYR停止移动,R去使能

void XYR_ZDT_Speed_Setting_VOFA(uint8_t addr, float velocity); // 张大头速度设置(针对VOFA+)
void XYR_HT_Speed_Setting_VOFA(uint8_t addr, float velocity);  // 转台速度设置(针对VOFA+)
void XYR_ZDT_Pos_Setting_VOFA(uint8_t addr, int32_t position); // 张大头位移距离设置(针对VOFA+)
void XYR_HT_Pos_Setting_VOFA(uint8_t addr, int32_t position);  // 转台位旋转角度设置(针对VOFA+)
float Parse_Float_LittleEndian(uint8_t *bytes);				   // 小端排序组合返回数值(针对VOFA+)
void XYR_AutoScan_Start(uint8_t x, uint8_t y);				   // 开始面扫(针对VOFA+)
void XYR_AutoScan_Stop(void);								   // 停止面扫(针对VOFA+)

void Controller_Update_Callback(void); // 指令发送端

void XYR_Send_USB_Commend(); // 串口发送状态更新
void XYR_Read_USB_Commend(); // 串口解析命令

void XYR_MotorState_Transition(uint8_t addr, XYR_State new_state);
void XYR_MotorState_Update(uint8_t addr);

extern __IO CAN_t can;
extern __IO CAN_t can2;
extern volatile bool moveFlag[3];
extern volatile float VOFA_Speed[3];
extern volatile float VOFA_Pos[3];
extern volatile bool AutoScanFlag;
extern volatile bool USB_Send_Flag;

extern volatile uint8_t xyr_request;

#endif // XYR_CONTROL_H