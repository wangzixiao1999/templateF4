#ifndef __XYR_CONTROL_H
#define __XYR_CONTROL_H

// XYR_Control.h
// XYR三轴位移台控制

#include "ZDT_X42_V2.h"
#include "HT_DM_S_7010.h"

void XYR_Init();
void XYR_Collision_Home(uint8_t addr);//碰撞回零
void XYR_Fixed_Length_Move(uint8_t addr, uint8_t dir, float velocity, float position);//定长移动

//中断函数使用
void Controller_Update_Callback(void);


extern __IO CAN_t can;
extern __IO CAN_t can2;
extern volatile bool moveFlag;

#endif // XYR_CONTROL_H