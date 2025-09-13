/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tec.c
  * @brief   This file provides code for the configuration
  *          of the TEC instances.
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
#include "tec.h"

/* USER CODE BEGIN 0 */

/*
- TEC_Init() ：初始化TEC控制
- TEC_Start() ：启动制冷
- TEC_Stop() ：停止制冷
- TEC_SetPower() ：设置制冷功率(0-100%)
- TEC_GetState() ：获取当前状态
- TEC_GetPower() ：获取当前功率
*/

/* USER CODE END 0 */

/* Private variables ---------------------------------------------------------*/
static TEC_HandleTypeDef htec;

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @brief  初始化TEC控制
  * @param  htim: PWM定时器句柄
  * @param  channel: PWM通道
  * @retval None
  */
void TEC_Init(TIM_HandleTypeDef* htim, uint32_t channel)
{
    /* USER CODE BEGIN 2 */
    htec.timHandle = htim;
    htec.channel = channel;
    htec.state = TEC_STATE_STOP;
    htec.duty = 0;
    /* USER CODE END 2 */
}

/**
  * @brief  启动TEC制冷
  * @param  None
  * @retval None
  */
void TEC_Start(void)
{
    /* USER CODE BEGIN 3 */
    if (htec.state == TEC_STATE_STOP) {
        HAL_TIM_PWM_Start(htec.timHandle, htec.channel);
        htec.state = TEC_STATE_RUNNING;
    }
    /* USER CODE END 3 */
}

/**
  * @brief  停止TEC制冷
  * @param  None
  * @retval None
  */
void TEC_Stop(void)
{
    /* USER CODE BEGIN 4 */
    if (htec.state == TEC_STATE_RUNNING) {
        HAL_TIM_PWM_Stop(htec.timHandle, htec.channel);
        htec.state = TEC_STATE_STOP;
        htec.duty = 0;
    }
    /* USER CODE END 4 */
}

/**
  * @brief  设置TEC制冷功率
  * @param  duty: 占空比 (0-100)
  * @retval None
  */
void TEC_SetPower(uint8_t duty)
{
    /* USER CODE BEGIN 5 */
    if (duty > 100) {
        duty = 100;
    }
    
    htec.duty = duty;
    
    // 计算比较值
    uint32_t pulse = (htec.timHandle->Init.Period + 1) * duty / 100;
    __HAL_TIM_SET_COMPARE(htec.timHandle, htec.channel, pulse);
    /* USER CODE END 5 */
}

/**
  * @brief  获取当前TEC状态
  * @param  None
  * @retval TEC状态
  */
TEC_State TEC_GetState(void)
{
    /* USER CODE BEGIN 6 */
    return htec.state;
    /* USER CODE END 6 */
}

/**
  * @brief  获取当前TEC功率
  * @param  None
  * @retval 当前占空比 (0-100)
  */
uint8_t TEC_GetPower(void)
{
    /* USER CODE BEGIN 7 */
    return htec.duty;
    /* USER CODE END 7 */
}

/* USER CODE BEGIN 8 */

/* USER CODE END 8 */