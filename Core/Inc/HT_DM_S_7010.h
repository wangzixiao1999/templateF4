#ifndef __HT_DM_S_7010_H
#define __HT_DM_S_7010_H

#include "can.h"

extern volatile int32_t HT_current; // 转台电流
extern volatile int32_t HT_speed;   // 转台速度
extern volatile int32_t HT_Single_circle_absolute_angle;   // 转台单圈绝对角度
extern volatile int32_t HT_Multi_circle_absolute_angle;   // 转台多圈绝对角度

typedef struct {
    int32_t current;
    int32_t speed;
    int32_t single_circle_angle;
    int32_t multi_circle_angle;
    uint8_t flag_current : 1;
    uint8_t flag_speed : 1;
    uint8_t flag_angle : 1;
} HT_DataCache;

// 全局缓存变量
static HT_DataCache HT_data_cache = {0};


/*******************************s***************************
*** HT_DM_S_7010 转台闭环控制
*** 编写作者：wzx
**********************************************************/

// 系统命令
void HT_DM_S_7010_System_Restart(uint8_t addr);                                    // 重启从机
void HT_DM_S_7010_Read_Version(uint8_t addr);                                       // 读版本信息
void HT_DM_S_7010_Read_Current_Q_Axis(uint8_t addr);                                // 读实时Q轴电流
void HT_DM_S_7010_Read_Speed(uint8_t addr);                                         // 读实时转速
void HT_DM_S_7010_Read_Absolute_Angle(uint8_t addr);                                // 读实时单圈/多圈绝对值角度
void HT_DM_S_7010_Read_Status_Fault(uint8_t addr);                                  // 读实时状态与故障码
void HT_DM_S_7010_Clear_Fault(uint8_t addr);                                        // 清除故障

// 参数配置命令
void HT_DM_S_7010_Read_Motor_Params(uint8_t addr);                                  // 读电机极对数、力矩常数、减速比
void HT_DM_S_7010_Set_Current_Position_As_Origin(uint8_t addr);                     // 设置当前位置为原点
void HT_DM_S_7010_Set_Position_Max_Speed(uint8_t addr, uint32_t max_speed);         // 设置位置模式最大转速
void HT_DM_S_7010_Set_Max_Q_Current(uint8_t addr, uint32_t max_current);            // 设置位置/速度模式最大Q轴电流
void HT_DM_S_7010_Set_Q_Current_Slope(uint8_t addr, uint32_t slope);                // 设置Q轴电流斜率
void HT_DM_S_7010_Set_Acceleration(uint8_t addr, uint32_t acceleration);            // 设置速度模式加速度
void HT_DM_S_7010_Set_Position_Kp(uint8_t addr, float kp);                          // 设置位置闭环Kp
void HT_DM_S_7010_Set_Position_Ki(uint8_t addr, float ki);                          // 设置位置闭环Ki
void HT_DM_S_7010_Set_Velocity_Kp(uint8_t addr, float kp);                          // 设置速度闭环Kp
void HT_DM_S_7010_Set_Velocity_Ki(uint8_t addr, float ki);                          // 设置速度闭环Ki

// 运动控制命令
void HT_DM_S_7010_Current_Control(uint8_t addr, int32_t current);                   // Q轴电流控制
void HT_DM_S_7010_Velocity_Control(uint8_t addr, int32_t speed);                    // 速度控制
void HT_DM_S_7010_Absolute_Position_Control(uint8_t addr, int32_t position);        // 绝对值位置控制
void HT_DM_S_7010_Relative_Position_Control(uint8_t addr, int32_t position);        // 相对位置控制
void HT_DM_S_7010_Return_To_Origin(uint8_t addr);                                   // 最短距离回原点
void HT_DM_S_7010_Brake_Control(uint8_t addr, uint8_t state);                       // 抱闸开关控制
void HT_DM_S_7010_Disable_Motor(uint8_t addr);                                      // 关闭电机输出

// 数据接收函数（用于解析从机应答）
void HT_DM_S_7010_Receive_Data(volatile uint8_t *rxCmd, volatile uint32_t *rxCount);                   // 返回数据接收函数
void HT_UpdateParameters(void);									// 更新全局参数函数

#endif /* __HT_DM_S_7010_H */