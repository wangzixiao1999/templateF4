#ifndef __XYR_CONTROL_H
#define __XYR_CONTROL_H

// XYR_Control.h
// XYR三轴位移台控制

#include "ZDT_X42_V2.h"
#include "HT_DM_S_7010.h"

void XYR_Init();
void XYR_Collision_Home(uint8_t addr);

//中断函数使用
void Controller_Update_Callback(void);


extern __IO CAN_t can;
extern __IO CAN_t can2;

#endif // XYR_CONTROL_H