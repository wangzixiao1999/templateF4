/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	__IO CAN_RxHeaderTypeDef CAN_RxMsg;
	__IO uint8_t rxData[32];

	__IO CAN_TxHeaderTypeDef CAN_TxMsg;
	__IO uint8_t txData[32];

	__IO bool rxFrameFlag;
}CAN_t;
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

extern __IO CAN_t can;
extern __IO CAN_t can2;

extern __IO HAL_StatusTypeDef SendState;

/* CAN online 状态与重试控制（在 can.c 中定义） */
extern volatile bool can1_online;
extern volatile bool can2_online;
extern uint32_t can1_last_try;
extern uint32_t can2_last_try;

/* 重试间隔（毫秒） */
#define CAN_RETRY_INTERVAL_MS 5000U

/* 在定时或主循环里调用：尝试启动/恢复 CAN */
void TryStartCAN(CAN_HandleTypeDef *hcan, volatile bool *online_flag, uint32_t *last_try_ts);
/* 简单接口：尝试恢复所有 CAN */
void CAN_TryStartAll(void);
/* 初始化重试时间戳，使首次尝试尽快发生 */
void CAN_InitRetryTimers(void);
/* 查询接口：1 表示 CAN1 在线，2 表示 CAN2 在线 */
bool CAN_IsOnline(uint8_t idx);

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

void USER_CAN1_Filter_Init(void);
bool can_SendCmd(__IO uint8_t *cmd, uint8_t len);

void USER_CAN2_Filter_Init(void);
bool can2_SendCmd(__IO uint8_t *cmd, uint8_t len);
bool can2_SendStdFrame(uint16_t std_id, const uint8_t *data, uint8_t len);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

