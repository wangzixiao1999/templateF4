#include "LK_Motor.h"

#include <string.h>

volatile LK_MotorFeedback g_lk_motor_feedback = {0};
volatile LK_MotorCommandQueue g_lk_motor_cmd = {0};

static uint16_t LK_Motor_BuildStdId(uint8_t id)
{
    return (uint16_t)(LK_MOTOR_STDID_BASE + id);
}

static bool LK_Motor_SendFrame(uint8_t id, const uint8_t *payload)
{
    return can2_SendStdFrame(LK_Motor_BuildStdId(id), payload, 8);
}

static void LK_Motor_ClearQueuedCmd(void)
{
    memset((void *)g_lk_motor_cmd.data, 0, sizeof(g_lk_motor_cmd.data));
    g_lk_motor_cmd.len = 0;
    g_lk_motor_cmd.pending = false;
}

static int64_t LK_Motor_ParseSigned56(const volatile uint8_t *bytes)
{
    int64_t value = 0;

    for (uint8_t i = 0; i < 7; ++i)
    {
        value |= ((int64_t)bytes[i] << (8U * i));
    }

    if ((value & (1LL << 55)) != 0)
    {
        value |= ~((1LL << 56) - 1);
    }

    return value;
}

void LK_Motor_Read_State1(uint8_t id)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x9A;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Clear_Error(uint8_t id)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x9B;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Read_State2(uint8_t id)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x9C;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Motor_Off(uint8_t id)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x80;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Motor_On(uint8_t id)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x88;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Stop(uint8_t id)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x81;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Brake_Control(uint8_t id, uint8_t brake_cmd)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x8C;
    frame[1] = brake_cmd;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Velocity_Control(uint8_t id, int16_t iq_limit, int32_t speed_0p01_dps)
{
    uint8_t frame[8] = {0};

    frame[0] = 0xA2;
    frame[2] = (uint8_t)(iq_limit >> 0);
    frame[3] = (uint8_t)(iq_limit >> 8);
    frame[4] = (uint8_t)(speed_0p01_dps >> 0);
    frame[5] = (uint8_t)(speed_0p01_dps >> 8);
    frame[6] = (uint8_t)(speed_0p01_dps >> 16);
    frame[7] = (uint8_t)(speed_0p01_dps >> 24);

    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Read_Multi_Turn_Angle(uint8_t id)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x92;
    LK_Motor_SendFrame(id, frame);
}

void LK_Motor_Load_Increment_Position_Control2(uint16_t max_speed_dps, int32_t angle_increment_0p01deg)
{
    g_lk_motor_cmd.data[0] = 0xA8;
    g_lk_motor_cmd.data[1] = 0x00;
    g_lk_motor_cmd.data[2] = (uint8_t)(max_speed_dps >> 0);
    g_lk_motor_cmd.data[3] = (uint8_t)(max_speed_dps >> 8);
    g_lk_motor_cmd.data[4] = (uint8_t)(angle_increment_0p01deg >> 0);
    g_lk_motor_cmd.data[5] = (uint8_t)(angle_increment_0p01deg >> 8);
    g_lk_motor_cmd.data[6] = (uint8_t)(angle_increment_0p01deg >> 16);
    g_lk_motor_cmd.data[7] = (uint8_t)(angle_increment_0p01deg >> 24);
    g_lk_motor_cmd.len = 8;
    g_lk_motor_cmd.pending = true;
}

void LK_Motor_Receive_Data(uint16_t std_id, const volatile uint8_t *rx_cmd, uint8_t rx_count)
{
    if (rx_cmd == NULL || rx_count == 0U)
    {
        return;
    }

    if (std_id != LK_Motor_BuildStdId(LK_MOTOR_ID))
    {
        return;
    }

    g_lk_motor_feedback.last_reply_cmd = rx_cmd[0];
    g_lk_motor_feedback.last_rx_tick = HAL_GetTick();

    switch (rx_cmd[0])
    {
    case 0x9A:
    case 0x9B:
        if (rx_count >= 8U)
        {
            g_lk_motor_feedback.temperature = (int8_t)rx_cmd[1];
            g_lk_motor_feedback.bus_voltage_centi_v = (int16_t)((uint16_t)rx_cmd[2] | ((uint16_t)rx_cmd[3] << 8));
            g_lk_motor_feedback.bus_current_centi_a = (int16_t)((uint16_t)rx_cmd[4] | ((uint16_t)rx_cmd[5] << 8));
            g_lk_motor_feedback.motor_state = rx_cmd[6];
            g_lk_motor_feedback.error_state = rx_cmd[7];
        }
        break;

    case 0x9C:
    case 0xA1:
    case 0xA2:
    case 0xA3:
    case 0xA4:
    case 0xA5:
    case 0xA6:
    case 0xA7:
    case 0xA8:
        if (rx_count >= 8U)
        {
            g_lk_motor_feedback.temperature = (int8_t)rx_cmd[1];
            g_lk_motor_feedback.iq_current_raw = (int16_t)((uint16_t)rx_cmd[2] | ((uint16_t)rx_cmd[3] << 8));
            g_lk_motor_feedback.speed_dps = (int16_t)((uint16_t)rx_cmd[4] | ((uint16_t)rx_cmd[5] << 8));
            g_lk_motor_feedback.encoder = (uint16_t)((uint16_t)rx_cmd[6] | ((uint16_t)rx_cmd[7] << 8));
        }
        break;

    case 0x92:
        if (rx_count >= 8U)
        {
            int64_t angle_0p01deg = LK_Motor_ParseSigned56(&rx_cmd[1]);
            g_lk_motor_feedback.multi_turn_angle_deg = (float)angle_0p01deg / 100.0f;
        }
        break;

    default:
        break;
    }
}

void LK_Motor_Push_Data_From_ISR(uint16_t std_id, const volatile uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0U || len > 8U)
    {
        return;
    }

    LK_Motor_Receive_Data(std_id, data, (uint8_t)len);
}

void LK_cmdSend(void)
{
    if ((g_lk_motor_cmd.pending == true) && (g_lk_motor_cmd.len == 8U))
    {
        if (LK_Motor_SendFrame(LK_MOTOR_ID, (const uint8_t *)g_lk_motor_cmd.data))
        {
            LK_Motor_ClearQueuedCmd();
        }
    }
}
