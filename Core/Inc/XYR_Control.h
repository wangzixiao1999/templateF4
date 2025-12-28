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

// 指令发送端
void Controller_Update_Callback(void);

extern __IO CAN_t can;
extern __IO CAN_t can2;
extern volatile bool moveFlag[3];
extern volatile float VOFA_Speed[3];
extern volatile float VOFA_Pos[3];
#endif // XYR_CONTROL_H