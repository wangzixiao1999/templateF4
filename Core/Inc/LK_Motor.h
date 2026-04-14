#ifndef __LK_MOTOR_H
#define __LK_MOTOR_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LK_MOTOR_ID 3U
#define LK_MOTOR_STDID_BASE 0x140U
#define LK_DEFAULT_IQ_LIMIT 1500

typedef struct
{
    int8_t temperature;
    int16_t iq_current_raw;
    int16_t speed_dps;
    uint16_t encoder;
    int16_t bus_voltage_centi_v;
    int16_t bus_current_centi_a;
    uint8_t motor_state;
    uint8_t error_state;
    float multi_turn_angle_deg;
    uint8_t last_reply_cmd;
    uint32_t last_rx_tick;
} LK_MotorFeedback;

typedef struct
{
    uint8_t data[8];
    uint8_t len;
    bool pending;
} LK_MotorCommandQueue;

extern volatile LK_MotorFeedback g_lk_motor_feedback;
extern volatile LK_MotorCommandQueue g_lk_motor_cmd;

void LK_Motor_Read_State1(uint8_t id);
void LK_Motor_Clear_Error(uint8_t id);
void LK_Motor_Read_State2(uint8_t id);
void LK_Motor_Motor_Off(uint8_t id);
void LK_Motor_Motor_On(uint8_t id);
void LK_Motor_Stop(uint8_t id);
void LK_Motor_Brake_Control(uint8_t id, uint8_t brake_cmd);
void LK_Motor_Velocity_Control(uint8_t id, int16_t iq_limit, int32_t speed_0p01_dps);
void LK_Motor_Read_Multi_Turn_Angle(uint8_t id);
void LK_Motor_Load_Single_Position_Control2(uint8_t spin_direction, uint16_t max_speed_dps, uint32_t angle_control_0p01deg);
void LK_Motor_Load_Increment_Position_Control2(uint16_t max_speed_dps, int32_t angle_increment_0p01deg);
void LK_Motor_Receive_Data(uint16_t std_id, const volatile uint8_t *rx_cmd, uint8_t rx_count);
void LK_Motor_Push_Data_From_ISR(uint16_t std_id, const volatile uint8_t *data, uint32_t len);
void LK_cmdSend(void);

#ifdef __cplusplus
}
#endif

#endif /* __LK_MOTOR_H */
