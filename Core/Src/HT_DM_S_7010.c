
#include "HT_DM_S_7010.h"
#include <string.h>

int32_t HT_current = 0;                      // 转台电流
int32_t HT_speed = 0;                        // 转台速度
int32_t HT_Single_circle_absolute_angle = 0; // 转台单圈绝对角度
int32_t HT_Multi_circle_absolute_angle = 0;  // 转台多圈绝对角度

float HTRev_freq = 0.f;

/**
 * @brief    重启从机，主控制器发送该命令包后，从机立即重启不应答主控制器
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码
 */
void HT_DM_S_7010_System_Restart(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0x00;
  cmd[2] = 0xFF;
  cmd[3] = 0x00;
  cmd[4] = 0xFF;
  cmd[5] = 0x00;
  cmd[6] = 0xFF;
  cmd[7] = 0x00;
  cmd[8] = 0xFF;

  can2_SendCmd((__IO uint8_t *)cmd, 9);
}

/**
 * @brief    读Boot、软件、硬件、自定义CAN协议版本
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Read_Version(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xA0;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    读实时Q轴电流
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Read_Current_Q_Axis(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xA1;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    读实时旋转速度
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Read_Speed(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xA2;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    读实时单圈绝对值角度、多圈绝对值角度
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Read_Absolute_Angle(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xA3;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
  * @brief    读取母线电压、母线电流、工作温度、运行模式及故障码状态信息；从机一旦检测到故
障，将以200ms时间周期上报实时状态信息
  * @param    addr  ：电机地址
  */
void HT_DM_S_7010_Read_Status_Fault(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xAE;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    清除故障
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Clear_Fault(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xAF;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/* ---------- 参数配置命令 ---------- */

/**
 * @brief    读取电机极对数、力矩常数、减速比
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Read_Motor_Params(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xB0;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    设置当前位置为原点
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Set_Current_Position_As_Origin(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xB1;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    设置位置模式旋转最大速度，断电不保存
 * @param    addr  ：电机地址
 * @param    max_speed  ：最大速度
 */
void HT_DM_S_7010_Set_Position_Max_Speed(uint8_t addr, uint32_t max_speed)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xB2;
  cmd[2] = (uint8_t)(max_speed >> 0);
  cmd[3] = (uint8_t)(max_speed >> 8);
  cmd[4] = (uint8_t)(max_speed >> 16);
  cmd[5] = (uint8_t)(max_speed >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    设置位置或速度模式最大Q轴电流，断电不保存
 * @param    addr  ：电机地址
 * @param    max_current  ：最大电流
 */
void HT_DM_S_7010_Set_Max_Q_Current(uint8_t addr, uint32_t max_current)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xB3;
  cmd[2] = (uint8_t)(max_current >> 0);
  cmd[3] = (uint8_t)(max_current >> 8);
  cmd[4] = (uint8_t)(max_current >> 16);
  cmd[5] = (uint8_t)(max_current >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    设置Q轴电流控制模式下的Q轴电流斜率，断电不保存
 * @param    addr  ：电机地址
 * @param    slope  ：电流斜率
 */
void HT_DM_S_7010_Set_Q_Current_Slope(uint8_t addr, uint32_t slope)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xB4;
  cmd[2] = (uint8_t)(slope >> 0);
  cmd[3] = (uint8_t)(slope >> 8);
  cmd[4] = (uint8_t)(slope >> 16);
  cmd[5] = (uint8_t)(slope >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    设置速度控制模式下的加速度，断电不保存
 * @param    addr  ：电机地址
 * @param    acceleration  ：加速度
 */
void HT_DM_S_7010_Set_Acceleration(uint8_t addr, uint32_t acceleration)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xB5;
  cmd[2] = (uint8_t)(acceleration >> 0);
  cmd[3] = (uint8_t)(acceleration >> 8);
  cmd[4] = (uint8_t)(acceleration >> 16);
  cmd[5] = (uint8_t)(acceleration >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    设置位置控制闭环Kp，断电不保存
 * @param    addr  ：电机地址
 * @param    kp  ：Kp值
 */
void HT_DM_S_7010_Set_Position_Kp(uint8_t addr, float kp)
{
  uint8_t cmd[12] = {0};
  union
  {
    float f;
    uint8_t b[4];
  } u;
  u.f = kp;
  cmd[0] = addr;
  cmd[1] = 0xB6;
  cmd[2] = u.b[0];
  cmd[3] = u.b[1];
  cmd[4] = u.b[2];
  cmd[5] = u.b[3];
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    设置位置控制闭环Ki，断电不保存
 * @param    addr  ：电机地址
 * @param    ki  ：Ki值
 */
void HT_DM_S_7010_Set_Position_Ki(uint8_t addr, float ki)
{
  uint8_t cmd[12] = {0};
  union
  {
    float f;
    uint8_t b[4];
  } u;
  u.f = ki;
  cmd[0] = addr;
  cmd[1] = 0xB7;
  cmd[2] = u.b[0];
  cmd[3] = u.b[1];
  cmd[4] = u.b[2];
  cmd[5] = u.b[3];
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    设置速度控制闭环Kp，断电不保存
 * @param    addr  ：电机地址
 * @param    kp  ：Kp值
 */
void HT_DM_S_7010_Set_Velocity_Kp(uint8_t addr, float kp)
{
  uint8_t cmd[12] = {0};
  union
  {
    float f;
    uint8_t b[4];
  } u;
  u.f = kp;
  cmd[0] = addr;
  cmd[1] = 0xB8;
  cmd[2] = u.b[0];
  cmd[3] = u.b[1];
  cmd[4] = u.b[2];
  cmd[5] = u.b[3];
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    设置速度控制闭环Ki，断电不保存
 * @param    addr  ：电机地址
 * @param    ki  ：Ki值
 */
void HT_DM_S_7010_Set_Velocity_Ki(uint8_t addr, float ki)
{
  uint8_t cmd[12] = {0};
  union
  {
    float f;
    uint8_t b[4];
  } u;
  u.f = ki;
  cmd[0] = addr;
  cmd[1] = 0xB9;
  cmd[2] = u.b[0];
  cmd[3] = u.b[1];
  cmd[4] = u.b[2];
  cmd[5] = u.b[3];
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    Q轴电流控制
 * @param    addr  ：电机地址
 * @param    current  ：Q轴电流
 */
void HT_DM_S_7010_Current_Control(uint8_t addr, int32_t current)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xC0;
  cmd[2] = (uint8_t)(current >> 0);
  cmd[3] = (uint8_t)(current >> 8);
  cmd[4] = (uint8_t)(current >> 16);
  cmd[5] = (uint8_t)(current >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    速度控制
 * @param    addr  ：电机地址
 * @param    speed  ：速度
 */
void HT_DM_S_7010_Velocity_Control(uint8_t addr, int32_t speed)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xC1;
  cmd[2] = (uint8_t)(speed >> 0);
  cmd[3] = (uint8_t)(speed >> 8);
  cmd[4] = (uint8_t)(speed >> 16);
  cmd[5] = (uint8_t)(speed >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    绝对值位置控制
 * @param    addr  ：电机地址
 * @param    position  ：绝对位置
 */
void HT_DM_S_7010_Absolute_Position_Control(uint8_t addr, int32_t position)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xC2;
  cmd[2] = (uint8_t)(position >> 0);
  cmd[3] = (uint8_t)(position >> 8);
  cmd[4] = (uint8_t)(position >> 16);
  cmd[5] = (uint8_t)(position >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    相对值位置控制
 * @param    addr  ：电机地址
 * @param    position  ：相对位置
 */
void HT_DM_S_7010_Relative_Position_Control(uint8_t addr, int32_t position)
{
  uint8_t cmd[12] = {0};
  cmd[0] = addr;
  cmd[1] = 0xC3;
  cmd[2] = (uint8_t)(position >> 0);
  cmd[3] = (uint8_t)(position >> 8);
  cmd[4] = (uint8_t)(position >> 16);
  cmd[5] = (uint8_t)(position >> 24);
  can2_SendCmd((__IO uint8_t *)cmd, 6);
}

/**
 * @brief    最短距离回原点
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Return_To_Origin(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xC4;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    抱闸开关输出控制
 * @param    addr  ：电机地址
 * @param    state ：抱闸状态，0为断开抱闸，1为闭合抱闸
 */
void HT_DM_S_7010_Brake_Control(uint8_t addr, uint8_t state)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xFE;
  cmd[2] = state;
  can2_SendCmd((__IO uint8_t *)cmd, 3);
}

/**
 * @brief    关闭电机输出
 * @param    addr  ：电机地址
 */
void HT_DM_S_7010_Disable_Motor(uint8_t addr)
{
  uint8_t cmd[8] = {0};
  cmd[0] = addr;
  cmd[1] = 0xEF;
  can2_SendCmd((__IO uint8_t *)cmd, 2);
}

/**
 * @brief    返回数据接收函数
 * @param    rxCmd   : 接收到的数据缓存在该数组
 * @param    rxCount : 接收到的数据长度
 */
void HT_DM_S_7010_Receive_Data(volatile uint8_t *rxCmd, volatile uint32_t *rxCount)
{
  if (rxCmd == NULL || rxCount == NULL)
    return;
  if (*rxCount == 0)
    return;

  uint8_t cmd = rxCmd[0];
  switch (cmd)
  {
  case 0xA1:
    if (*rxCount >= 5)
    {
      HT_data_cache.current = (int32_t)rxCmd[1] | ((int32_t)rxCmd[2] << 8) | ((int32_t)rxCmd[3] << 16) | ((int32_t)rxCmd[4] << 24);
      HT_data_cache.flag_current = 1;
      HT_UpdateParameters();
    }
    break;
  case 0xA2:
    if (*rxCount >= 5)
    {
      HT_data_cache.speed = (int32_t)rxCmd[1] | ((int32_t)rxCmd[2] << 8) | ((int32_t)rxCmd[3] << 16) | ((int32_t)rxCmd[4] << 24);
      HT_data_cache.flag_speed = 1;
      HT_UpdateParameters();
    }
    break;
  case 0xA3:
    if (*rxCount >= 7)
    {
      HT_data_cache.single_circle_angle = (int32_t)rxCmd[1] | ((int32_t)rxCmd[2] << 8);
      HT_data_cache.multi_circle_angle = (int32_t)rxCmd[3] | ((int32_t)rxCmd[4] << 8) | ((int32_t)rxCmd[5] << 16) | ((int32_t)rxCmd[6] << 24);
      HT_data_cache.flag_angle = 1;
      HT_UpdateParameters();
    }
    break;
  }
}

/**
  * @brief    更新全局参数
  * @retval   无
  */
void HT_UpdateParameters(void)
{
  static volatile uint32_t preTick_HT = 0;

  if (HT_data_cache.flag_current && HT_data_cache.flag_speed && HT_data_cache.flag_angle)
  {

    HT_current = HT_data_cache.current;
    HT_speed = HT_data_cache.speed;
    HT_Single_circle_absolute_angle = HT_data_cache.single_circle_angle;
    HT_Multi_circle_absolute_angle = HT_data_cache.multi_circle_angle;

    HT_data_cache.flag_current = 0;
    HT_data_cache.flag_speed = 0;
    HT_data_cache.flag_angle = 0;

    uint32_t currTick_HT = HAL_GetTick();
    HTRev_freq = 1000.f / (currTick_HT - preTick_HT);
    preTick_HT = currTick_HT;
  }
}