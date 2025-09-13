/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tec.h
  * @brief   This file contains all the function prototypes for
  *          the tec.c file
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
#ifndef __TEC_H__
#define __TEC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    TEC_STATE_STOP,     // 停止状态
    TEC_STATE_RUNNING   // 运行状态
} TEC_State;

typedef struct {
    TIM_HandleTypeDef* timHandle;   // PWM定时器句柄
    uint32_t channel;               // PWM通道
    TEC_State state;                // 当前状态
    uint8_t duty;                   // 当前占空比 (0-100)
} TEC_HandleTypeDef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void TEC_Init(TIM_HandleTypeDef* htim, uint32_t channel);
void TEC_Start(void);
void TEC_Stop(void);
void TEC_SetPower(uint8_t duty);
TEC_State TEC_GetState(void);
uint8_t TEC_GetPower(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __TEC_H__ */