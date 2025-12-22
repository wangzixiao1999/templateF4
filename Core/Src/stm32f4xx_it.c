/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"

extern volatile uint32_t tstart_ticks[4];
extern volatile uint32_t twidth_ticks[4];
extern volatile uint8_t pulse_state[4];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
float can_freq = 0.f;
float can2_freq = 0.f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
  uint8_t i = 0;
  static volatile uint32_t preTick = 0;
	// 接收�?包数�?
	if(HAL_CAN_GetRxMessage((&hcan1), CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *)(&can.CAN_RxMsg), (uint8_t *)(&can.rxData)) == HAL_OK)
	{
		// �?帧数据接收完成，置位帧标志位
		for(i=can.CAN_RxMsg.DLC; i < 8; i++) { can.rxData[i] = 0; } can.rxFrameFlag = true;
	}
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
  // 计算CAN接收频率
  uint32_t currTick = HAL_GetTick();
  can_freq = 1000.f / (currTick - preTick);
  preTick = currTick;

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
     /* Channel 1 */
    if (TIM1->SR & TIM_SR_CC1IF)
    {
        TIM1->SR &= ~TIM_SR_CC1IF; /* ??? */
        switch (pulse_state[0])
        {
          case 1:
          {
              TIM1->CCR1 = tstart_ticks[0] + twidth_ticks[0];
              pulse_state[0] = 2;
              break;
          }
          case 2:
          {
              HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
              pulse_state[0] = 0;
              break;
          }
          default:
          {
              HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
              pulse_state[0] = 0;
              break;
          }
        }
    }

    /* Channel 2 */
    if (TIM1->SR & TIM_SR_CC2IF)
    {
        TIM1->SR &= ~TIM_SR_CC2IF;
        switch (pulse_state[1])
        {
          case 1:
          {
            TIM1->CCR2 = tstart_ticks[1] + twidth_ticks[1];
            pulse_state[1] = 2;
            break;
          }
          case 2:
          {
            HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
            pulse_state[1] = 0;
            break;
          }
          default:
          {
            HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
            pulse_state[1] = 0;
            break;
          }
        }
    }

    /* Channel 3 */
    if (TIM1->SR & TIM_SR_CC3IF)
    {
        TIM1->SR &= ~TIM_SR_CC3IF;
        switch (pulse_state[2])
        {
          case 1:
          {
            TIM1->CCR3 = tstart_ticks[2] + twidth_ticks[2];
            pulse_state[2] = 2;
            break;
          }
          case 2:
          {
            HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
            pulse_state[2] = 0;
            break;
          }
          default:
          {
            HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
            pulse_state[2] = 0;
            break;
          }
        }
    }
    /* Channel 4 */
    if (TIM1->SR & TIM_SR_CC4IF)
    {
        TIM1->SR &= ~TIM_SR_CC4IF;
        switch (pulse_state[3])
        {
          case 1:
          {
            TIM1->CCR4 = tstart_ticks[3] + twidth_ticks[3];
            pulse_state[3] = 2;
            break;
          }
          case 2:
          {
            HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
            pulse_state[3] = 0;
            break;
          }
          default:
          {
            HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
            pulse_state[3] = 0;
            break;
          }
        }
    }
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
  uint8_t i = 0;
  static volatile uint32_t preTick = 0;

	// 接收�?包数�?
	if(HAL_CAN_GetRxMessage((&hcan2), CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *)(&can2.CAN_RxMsg), (uint8_t *)(&can2.rxData)) == HAL_OK)
	{
		//?帧数据接收完成，置位帧标志位
		for(i=can2.CAN_RxMsg.DLC; i < 8; i++) { can2.rxData[i] = 0; } can2.rxFrameFlag = true;
	}
  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */
  // 计算CAN2接收频率
  uint32_t currTick = HAL_GetTick();
  can2_freq = 1000.f / (currTick - preTick);
  preTick = currTick;

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
