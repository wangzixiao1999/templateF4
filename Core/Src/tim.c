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

volatile uint32_t tstart_ticks[4];
volatile uint32_t twidth_ticks[4];
volatile uint8_t pulse_state[4] = {0,0,0,0};

/* Buffers for DMA transfer (End Times) */
uint32_t tim1_end_val_buff[4];
DMA_HandleTypeDef hdma_tim1[4]; /* Ch1, Ch2, Ch3, Ch4 */

static uint32_t tim1_clk_hz = 0; /* 缓存 TIM1 时钟（Hz） */

static inline uint32_t us_to_ticks(double us)
{
    if (tim1_clk_hz == 0) {
        /* 保险：如果未初始化，就调用 HAL 接口尝试一次（但推荐在 main 中先调用 TIM1_ClockInit_Cache） */
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        if (pclk2 != 0) {
            /* 如果 HAL 返回非零，决策是否乘2 取决于 ppre2 */
            uint32_t tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
            if (tmp >= 4) tim1_clk_hz = pclk2 * 2;
            else tim1_clk_hz = pclk2;
        } else {
            /* 最后备方案：用 SystemCoreClock 假设 APB2=1（保守策略） */
            tim1_clk_hz = SystemCoreClock;
        }
    }
    /* 计算 ticks，避免浮点精度问题，使用 64-bit 中间 */
    uint64_t ticks = (uint64_t)(us * (double)tim1_clk_hz / 1e6 + 0.5);
    if (ticks > 0xFFFF) ticks = 0xFFFF; /* TIM1 CCR is 16-bit effective usually, unless F4 has 32-bit CC? F407 TIM1 is 16-bit. */
    return (uint32_t)ticks;
}

/* Helper to init DMA for a specific channel */
static void TIM1_DMA_Config_Channel(uint32_t Channel, DMA_Stream_TypeDef* Stream, uint32_t ChSel, int idx)
{
    /* Enable DMA2 Clock */
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_tim1[idx].Instance = Stream;
    hdma_tim1[idx].Init.Channel = ChSel;
    hdma_tim1[idx].Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1[idx].Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1[idx].Init.MemInc = DMA_MINC_DISABLE;
    hdma_tim1[idx].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* CCR 32-bit access safe */
    hdma_tim1[idx].Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1[idx].Init.Mode = DMA_NORMAL;
    hdma_tim1[idx].Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_tim1[idx].Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    if (HAL_DMA_Init(&hdma_tim1[idx]) != HAL_OK)
    {
       Error_Handler();
    }

    /* Link DMA to TIM Handle */
    switch(Channel) {
        case TIM_CHANNEL_1: __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], hdma_tim1[idx]); break;
        case TIM_CHANNEL_2: __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC2], hdma_tim1[idx]); break;
        case TIM_CHANNEL_3: __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC3], hdma_tim1[idx]); break;
        case TIM_CHANNEL_4: __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC4], hdma_tim1[idx]); break;
    }
}

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

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
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
/* TIM4 init function */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
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
    GPIO_InitStruct.Pull = GPIO_NOPULL;
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

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /* TIM4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void start_4_one_shot(double delays_us[4], double widths_us[4]) {
    /* 1. Disable IRQ to prevent interference */
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);

    /* 2. Configure DMA Streams if not already configured (First run only or re-config safe) */
    /* TIM1_CH1 -> DMA2 Stream 1 Channel 6 */
    TIM1_DMA_Config_Channel(TIM_CHANNEL_1, DMA2_Stream1, DMA_CHANNEL_6, 0);
    /* TIM1_CH2 -> DMA2 Stream 2 Channel 6 */
    TIM1_DMA_Config_Channel(TIM_CHANNEL_2, DMA2_Stream2, DMA_CHANNEL_6, 1);
    /* TIM1_CH3 -> DMA2 Stream 6 Channel 6 */
    TIM1_DMA_Config_Channel(TIM_CHANNEL_3, DMA2_Stream6, DMA_CHANNEL_6, 2);
    /* TIM1_CH4 -> DMA2 Stream 4 Channel 6 */
    TIM1_DMA_Config_Channel(TIM_CHANNEL_4, DMA2_Stream4, DMA_CHANNEL_6, 3);

    /* 3. Calculate Ticks and Prepare DMA Buffers */
    uint32_t max_end_tick = 0;
    
    for (int i=0; i<4; i++){
        uint32_t start = us_to_ticks(delays_us[i]);
        uint32_t width = us_to_ticks(widths_us[i]);
        uint32_t end = start + width;
        
        /* Store End Time in DMA Buffer */
        tim1_end_val_buff[i] = end;

        /* Track max tick for ARR */
        if (end > max_end_tick) max_end_tick = end;

        /* Set CCR to Start Time (Rising Edge) */
        switch(i) {
            case 0: TIM1->CCR1 = start; break;
            case 1: TIM1->CCR2 = start; break;
            case 2: TIM1->CCR3 = start; break;
            case 3: TIM1->CCR4 = start; break;
        }
    }

    /* 4. Configure Timer */
    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    
    /* Set ARR to Max End Time + Margin to allow last pulse to finish before update */
    /* One Pulse Mode stops at next Update Event. Update happens at overflow (CNT=ARR) or manual GEN. */
    /* We want CNT to go from 0 -> Max_End -> ARR -> Stop. */
    TIM1->ARR = max_end_tick + 100; 

    /* Enable One Pulse Mode (OPM) */
    TIM1->CR1 |= TIM_CR1_OPM;

    /* Clear Flags */
    TIM1->SR = 0;

    /* 5. Start DMA for each channel */
    /* This sets up the DMA to transfer 1 word (End Time) to CCR when CC flag rises */
    HAL_DMA_Start(&hdma_tim1[0], (uint32_t)&tim1_end_val_buff[0], (uint32_t)&TIM1->CCR1, 1);
    HAL_DMA_Start(&hdma_tim1[1], (uint32_t)&tim1_end_val_buff[1], (uint32_t)&TIM1->CCR2, 1);
    HAL_DMA_Start(&hdma_tim1[2], (uint32_t)&tim1_end_val_buff[2], (uint32_t)&TIM1->CCR3, 1);
    HAL_DMA_Start(&hdma_tim1[3], (uint32_t)&tim1_end_val_buff[3], (uint32_t)&TIM1->CCR4, 1);

    /* Enable TIM DMA Requests */
    /* Note: TIM_DMA_CCx enables DMA request on Compare Match */
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC2);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);

    /* 6. Enable Outputs (if not already enabled by HAL_TIM_OC_Start) */
    /* We use CCER register directly to ensure fast simultaneous start */
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    /* 7. Start Timer */
    /* MOE bit must be set for Advanced Timers (TIM1/TIM8) to output */
    TIM1->BDTR |= TIM_BDTR_MOE; 
    __HAL_TIM_ENABLE(&htim1);
}

/* 调用位置：在 SystemClock_Config() 完成后调用一次 */
void TIM1_ClockInit_Cache(void)
{
    /* 确保 SystemCoreClock 已被更新 */
    SystemCoreClockUpdate();
    uint32_t hclk = SystemCoreClock;

    /* 读取 APB2 prescaler 字段（bits 13:11）*/
    uint32_t tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    uint32_t apb2_div;
    switch (tmp) {
        case 0: case 1: case 2: case 3: apb2_div = 1; break; /* 0xx = /1 */
        case 4: apb2_div = 2; break;  /* 100 -> /2 */
        case 5: apb2_div = 4; break;  /* 101 -> /4 */
        case 6: apb2_div = 8; break;  /* 110 -> /8 */
        case 7: apb2_div = 16; break; /* 111 -> /16 */
        default: apb2_div = 1; break;
    }

    uint32_t pclk2 = hclk / apb2_div;
    if (apb2_div == 1) tim1_clk_hz = pclk2;
    else tim1_clk_hz = pclk2 * 2; /* F4 家族：APB prescaler !=1 时定时器时钟 = PCLK * 2 */
}

/* USER CODE END 1 */
