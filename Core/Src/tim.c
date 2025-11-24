/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

PulseChannel_t channels[4]; // 4个通道
uint32_t system_ticks = 0;  // 系统滴答计数

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void schedule_channel(uint8_t ch, float delay_us, float freq_Hz, float width_us, uint32_t pulse_count) {
  if(ch >= 4) return;

  /* 以 168 MHz 为基准：ticks_per_us = 168 */
  const float ticks_per_us = 168.0f;
  float period_us = (freq_Hz > 0.0f) ? (1000000.0f / freq_Hz) : 0.0f;

  /* 计算 ticks，边界检查到 16-bit */
  uint32_t delay_ticks = (uint32_t)roundf(delay_us * ticks_per_us);
  uint32_t period_ticks = (uint32_t)roundf(period_us * ticks_per_us);
  uint32_t width_ticks = (uint32_t)roundf(width_us * ticks_per_us);
  if (delay_ticks == 0) delay_ticks = 1; /* 保证非零延迟可以及时触发 */
  if (width_ticks == 0) width_ticks = 1;
  if (period_ticks == 0 && freq_Hz > 0.0f) period_ticks = width_ticks; /* 防止周期小于宽度 */

  if (delay_ticks > 0xFFFFFFFFu) delay_ticks = 0xFFFFFFFFu; /* 安全 */
  if (period_ticks > 0xFFFFu) period_ticks = 0xFFFFu; /* 将在触发前被截断 */
  if (width_ticks > 0xFFFFu) width_ticks = 0xFFFFu;

  channels[ch].delay_ticks = delay_ticks;
  channels[ch].period_ticks = period_ticks;
  channels[ch].width_ticks = width_ticks;
  channels[ch].count = pulse_count;
  channels[ch].current_count = 0;
  /* next_pulse_time 基于系统_ticks（TIM1 自由计数），保证后面比较有效 */
  channels[ch].next_pulse_time = system_ticks + channels[ch].delay_ticks;
  channels[ch].active = 1;

  /* 预写入 CCR 的阴影寄存器（写 CCRx 会更新阴影或直接寄存器，Update 时装载） */
  switch(ch) {
    case 0: htim1.Instance->CCR1 = channels[ch].width_ticks; break;
    case 1: htim1.Instance->CCR2 = channels[ch].width_ticks; break;
    case 2: htim1.Instance->CCR3 = channels[ch].width_ticks; break;
    case 3: htim1.Instance->CCR4 = channels[ch].width_ticks; break;
  }
}

/* update_pulse_generator: 需要被周期性调用（建议放到主循环或 SysTick/HAL_TIM 中断） */
void update_pulse_generator(void) {
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim1);
  system_ticks = now;

  uint8_t need_mask = 0;
  uint32_t max_period = 0;

  /* 为了在 start_synchronized_pulse 返回后更新计数，我们先收集需要触发的通道索引 */
  int need_indices[4];
  int need_count = 0;

  for (int i = 0; i < 4; ++i) {
    if (channels[i].active && channels[i].current_count < channels[i].count) {
      /* 环形比较（考虑 16-bit 溢出） */
      uint32_t next_time = channels[i].next_pulse_time;
      uint32_t diff;

      if (now >= next_time) {
        // 正常情况：当前时间 >= 下次触发时间
        diff = now - next_time;
      } else {
        // 定时器溢出情况：当前时间 < 下次触发时间（由于计数器从0重新开始）
        diff = (0xFFFF - next_time) + now + 1;
      }

      if (diff < 0x8000u) {
        need_mask |= (1u << i);
        need_indices[need_count++] = i;
        uint32_t p = channels[i].period_ticks;
        if (p == 0) p = channels[i].width_ticks;
        if (p > max_period) max_period = p;
        /* 注意：不在这里更新 current_count/next_pulse_time，等待脉冲完成后再更新 */
      }
    }
  }

  if (need_mask == 0) return;

  if (max_period == 0) max_period = 1;
  if (max_period > 0xFFFFu) max_period = 0xFFFFu;

  /* 启动同步单次脉冲，函数会阻塞直到该脉冲完成 */
  start_synchronized_pulse(max_period, need_mask);

  /* 脉冲完成后，用定时器当前 CNT 作为新的基准来更新 next_pulse_time */
  uint32_t end_now = __HAL_TIM_GET_COUNTER(&htim1);
  system_ticks = end_now;

  for (int k = 0; k < need_count; ++k) {
    int i = need_indices[k];
    channels[i].current_count++;
    channels[i].next_pulse_time = end_now + channels[i].period_ticks;
    if (channels[i].next_pulse_time > 0xFFFF) {
      channels[i].next_pulse_time &= 0xFFFF;
    }
    if (channels[i].current_count >= channels[i].count) {
      channels[i].active = 0;
    }
  }
}

/* start_synchronized_pulse：把每个需要触发的通道的 CCR 写入为 channels[i].width_ticks，
   然后把 TIM1 以 One-Pulse 模式启动一次（从 CNT=0 开始计数），保证所有通道的上升沿硬同步。
   触发完成后等待计数结束并停止 PWM 通道，确保输出回到空闲电平（防止一直为高）。 */
void start_synchronized_pulse(uint32_t period_ticks, uint32_t channel_mask) {
  /* 停止计数器（保证可以把 CNT 归零并安全写入 ARR/CCR） */
  __HAL_TIM_DISABLE(&htim1); /* 清 CEN */

  /* 清 CNT，准备从零开始计数，使 PWM 相位一致 */
  htim1.Instance->CNT = 0;

  /* 截断到 16-bit */
  if (period_ticks == 0) period_ticks = 1;
  if (period_ticks > 0xFFFFu) period_ticks = 0xFFFFu;
  htim1.Instance->ARR = (uint16_t)period_ticks;

  /* 为需要输出的通道写入各自的 CCR（使用 shadow，随后 UG 装载）；
     不需要触发的通道设置为 0（确保不输出） */
  if (channel_mask & (1u << 0)) {
    htim1.Instance->CCR1 = (uint16_t)channels[0].width_ticks;
  } else {
    htim1.Instance->CCR1 = 0;
  }
  if (channel_mask & (1u << 1)) {
    htim1.Instance->CCR2 = (uint16_t)channels[1].width_ticks;
  } else {
    htim1.Instance->CCR2 = 0;
  }
  if (channel_mask & (1u << 2)) {
    htim1.Instance->CCR3 = (uint16_t)channels[2].width_ticks;
  } else {
    htim1.Instance->CCR3 = 0;
  }
  if (channel_mask & (1u << 3)) {
    htim1.Instance->CCR4 = (uint16_t)channels[3].width_ticks;
  } else {
    htim1.Instance->CCR4 = 0;
  }

  /* 使能高级定时器主输出（MOE），必要时打开 BDTR 的 MOE 位 */
#ifdef __HAL_TIM_MOE_ENABLE
  __HAL_TIM_MOE_ENABLE(&htim1);
#else
  htim1.Instance->BDTR |= TIM_BDTR_MOE;
#endif

  /* 产生 Update 事件，立即把 CCR/ARR 的阴影装载到活动寄存器（硬件同步） */
  htim1.Instance->EGR = TIM_EGR_UG;

  /* 进入 One-Pulse 模式：计数完成后自动停止 */
  htim1.Instance->CR1 |= TIM_CR1_OPM;

  /* 启动需要的 PWM 通道输出（只启用需要的通道） */
  if (channel_mask & (1u << 0)) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  if (channel_mask & (1u << 1)) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  if (channel_mask & (1u << 2)) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  if (channel_mask & (1u << 3)) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* 启动计数器（从 CNT=0 开始），计数到 ARR 后因 OPM 自动停止（CEN 清零） */
  htim1.Instance->CR1 |= TIM_CR1_CEN;

  /* 等待 One-Pulse 完成：轮询直到定时器停止（OPM 会在溢出/ARR 后清 CEN） */
  while (htim1.Instance->CR1 & TIM_CR1_CEN) {
    __NOP();
  }

  /* 脉冲已完成：停止所有 PWM 通道输出，确保引脚返回到空闲电平 */
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

  /* 清除 One-Pulse 模式位（可选，保持计时器回到普通模式） */
  htim1.Instance->CR1 &= ~TIM_CR1_OPM;

  /* 取消主输出使能（MOE），可选：保持关闭以确保输出被硬件禁用 */
#ifdef __HAL_TIM_MOE_DISABLE
  __HAL_TIM_MOE_DISABLE(&htim1);
#else
  htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
#endif

}

/* 新增：阻塞运行一次，直到所有已设置的脉冲完成
   这是最小改动实现“调用一次函数，按设定时序/频率/脉冲数精确同步触发并等待完成”的方法。
   它只是循环调用 update_pulse_generator()，直到 channels 全部 inactive。
*/
void run_once_blocking(void) {
  /* 等待所有通道完成 */
  while (1) {
    uint8_t any_active = 0;
    for (int i = 0; i < 4; ++i) {
      if (channels[i].active && channels[i].current_count < channels[i].count) {
        any_active = 1;
        break;
      }
    }
    if (!any_active) break;

    /* 触发可能到时的脉冲（内部会阻塞直到 One-Pulse 完成）*/
    update_pulse_generator();

    /* 可选：短等待以避免过度占用 CPU（update_pulse_generator 内部已经使用定时器计数判断） */
    __NOP();
  }
}

/* USER CODE END 1 */
