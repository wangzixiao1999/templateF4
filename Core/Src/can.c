/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

__IO CAN_t can = {0};
__IO CAN_t can2 = {0};

__IO HAL_StatusTypeDef SendState = -1;

/* CAN 状态与重试时间戳（在此定义） */
volatile bool can1_online = false;
volatile bool can2_online = false;
uint32_t can1_last_try = 0;
uint32_t can2_last_try = 0;

/* 尝试间隔，定义在 can.h 中为 CAN_RETRY_INTERVAL_MS */

/*
 * TryStartCAN: 非阻塞尝试启动单个 CAN 控制器。
 * - 如果已在线直接返回
 * - 每次尝试之间至少间隔 CAN_RETRY_INTERVAL_MS
 * - 成功后激活接收中断，否则保持离线并等待下次重试
 */
void TryStartCAN(CAN_HandleTypeDef *hcan, volatile bool *online_flag, uint32_t *last_try_ts)
{
  uint32_t now = HAL_GetTick();
  if (*online_flag) return;
  if (now - *last_try_ts < CAN_RETRY_INTERVAL_MS) return;
  *last_try_ts = now;

  if (HAL_CAN_Start(hcan) == HAL_OK)
  {
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
    {
      *online_flag = true;
    }
    else
    {
      HAL_CAN_Stop(hcan);
      *online_flag = false;
    }
  }
  else
  {
    *online_flag = false;
  }
}

/*
 * CAN_TryStartAll: 便利函数，在周期任务中调用以尝试恢复 CAN1/CAN2
 */
void CAN_TryStartAll(void)
{
  TryStartCAN(&hcan1, &can1_online, &can1_last_try);
  TryStartCAN(&hcan2, &can2_online, &can2_last_try);
}

/*
 * CAN_InitRetryTimers: 设置重试时间戳使得首次尝试尽快发生
 */
void CAN_InitRetryTimers(void)
{
  uint32_t now = HAL_GetTick();
  can1_last_try = now - CAN_RETRY_INTERVAL_MS;
  can2_last_try = now - CAN_RETRY_INTERVAL_MS;
}

/*
 * CAN 状态查询
 */
bool CAN_IsOnline(uint8_t idx)
{
  if (idx == 1) return can1_online;
  if (idx == 2) return can2_online;
  return false;
}

/**
 * HAL CAN 错误回调：标记对应 CAN 为离线并停止它，延后重试
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t now = HAL_GetTick();
  if (hcan == &hcan1)
  {
    can1_online = false;
    HAL_CAN_Stop(&hcan1);
    can1_last_try = now;
  }
  else if (hcan == &hcan2)
  {
    can2_online = false;
    HAL_CAN_Stop(&hcan2);
    can2_last_try = now;
  }
  /* 可选：记录错误码 HAL_CAN_GetError(hcan) */
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 7;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief   初始化滤波器
 * @param   �?
 * @retval  �?
 */
void USER_CAN1_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;
  uint8_t id_o = 0x00, im_o = 0x00;
  uint16_t id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_ID_EXT;
  uint16_t id_h = (uint16_t)((uint16_t)id_o >> 5);
  uint16_t im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_ID_EXT;
  uint16_t im_h = (uint16_t)((uint16_t)im_o >> 5);

  sFilterConfig.FilterBank = 0; // CAN1 使用 bank 0 起始
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = id_h;
  sFilterConfig.FilterIdLow = id_l;
  sFilterConfig.FilterMaskIdHigh = im_h;
  sFilterConfig.FilterMaskIdLow = im_l;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14; // 关键：告诉 CAN1，bank >=14 分配给 CAN2

  while (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    ;
}

void USER_CAN2_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;
  uint8_t id_o = 0x00, im_o = 0x00;
  uint16_t id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_ID_STD;
  uint16_t id_h = (uint16_t)((uint16_t)id_o >> 5);
  uint16_t im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_ID_STD;
  uint16_t im_h = (uint16_t)((uint16_t)im_o >> 5);

  sFilterConfig.FilterBank = 14; // CAN2 使用从 14 开始的 bank
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = id_h;
  sFilterConfig.FilterIdLow = id_l;
  sFilterConfig.FilterMaskIdHigh = im_h;
  sFilterConfig.FilterMaskIdLow = im_l;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0; // 对 CAN2 无效，但填个值

  while (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    ;
}

/**
 * @brief   通过 CAN1 发送命令
 * @param   cmd: 命令数据指针
 * @param   len: 命令长度
 */
bool can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
  uint32_t TxMailbox;
  uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;

  // 除去ID地址和功能码后的数据长度
  j = len - 2;

  // 发包
  while (i < j)
  {
    // 数据个数
    k = j - i;

    // 填充缓存
    can.CAN_TxMsg.StdId = 0x00;
    can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
    can.txData[0] = cmd[1];
    can.CAN_TxMsg.IDE = CAN_ID_EXT;
    can.CAN_TxMsg.RTR = CAN_RTR_DATA;

    // 小于8字节命令
    if (k < 8)
    {
      for (l = 0; l < k; l++, i++)
      {
        can.txData[l + 1] = cmd[i + 2];
      }
      can.CAN_TxMsg.DLC = k + 1;
    }
    // 大于8字节命令，分包发送，每包数据最多发 8 个字节
    else
    {
      for (l = 0; l < 7; l++, i++)
      {
        can.txData[l + 1] = cmd[i + 2];
      }
      can.CAN_TxMsg.DLC = 8;
    }

    // 发送数据，带超时保护：如果在调试中断或总线不可用时不致于无限阻塞
    uint32_t start = HAL_GetTick();
    const uint32_t tx_timeout_ms = 200; // 每包最多等待 200ms

    while (SendState = HAL_CAN_AddTxMessage((&hcan1), (CAN_TxHeaderTypeDef *)(&can.CAN_TxMsg), (uint8_t *)(&can.txData), (&TxMailbox)) != HAL_OK)
    {
      if ((HAL_GetTick() - start) >= tx_timeout_ms)
      {
        // 超时则放弃此次发送，返回失败
        return false;
      }
    }

    // 记录发的第几包的数据
    ++packNum;
  }

  return true;
}

/**
 * @brief   通过 CAN2 发送命令
 * @param   cmd: 命令数据指针
 * @param   len: 命令长度
 */
bool can2_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
  uint32_t TxMailbox;

  can2.CAN_TxMsg.StdId = ((uint32_t)cmd[0]) & 0x7FF;
  can2.CAN_TxMsg.IDE = CAN_ID_STD;
  can2.CAN_TxMsg.RTR = CAN_RTR_DATA;
  can2.CAN_TxMsg.DLC = len - 1;

  for (int i = 0; i < len - 1; i++)
  {
    can2.txData[i] = cmd[i + 1];
  }

  uint32_t start = HAL_GetTick();
  const uint32_t tx_timeout_ms = 200;
  while (HAL_CAN_AddTxMessage((&hcan2), (CAN_TxHeaderTypeDef *)(&can2.CAN_TxMsg), (uint8_t *)(&can2.txData), (&TxMailbox)) != HAL_OK)
  {
    if ((HAL_GetTick() - start) >= tx_timeout_ms)
    {
      return false;
    }
  }

  return true;
}
/* USER CODE END 1 */
