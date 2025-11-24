/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <math.h>

typedef struct {
  uint32_t delay_ticks;
  uint32_t period_ticks;
  uint32_t width_ticks;
  uint32_t count;
  uint32_t current_count;
  uint32_t next_pulse_time;
  uint8_t active;
} PulseChannel_t;

extern TIM_HandleTypeDef htim1;

void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* 修正：第二个参数为 channel_mask（位掩码），而非 width_ticks */
void schedule_channel(uint8_t ch, float delay_us, float freq_Hz, float width_us, uint32_t pulse_count);
void update_pulse_generator(void);
void start_synchronized_pulse(uint32_t period_ticks, uint32_t channel_mask);

/* （可选）如果你添加了阻塞运行一次的函数，也在此处声明 */
void run_once_blocking(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */