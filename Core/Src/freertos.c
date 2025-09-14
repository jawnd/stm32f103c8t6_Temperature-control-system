/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DS18B20.h"
#include "PID.h"
#include "math.h"
#include <stdio.h>
#include "tim.h"
#include "usart.h"
#include "stm32f1xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 全局变量用于任务间通信
float currentTemperature = 0.0;     // 当前温度
float targetTemperature = 25.0;     // 目标温度
uint8_t tecPower = 0;               // TEC功率输出
uint8_t tecEnabled = 1;             // TEC使能标志
char displayBuffer[64];             // 显示缓冲区

// PID控制器
PIDController pidController;

// 互斥锁用于保护共享数据
osMutexId_t temperatureMutexHandle;
const osMutexAttr_t temperatureMutex_attributes = {
  .name = "temperatureMutex"
};

/* USER CODE END Variables */
/* Definitions for UserControl_0 */
osThreadId_t UserControl_0Handle;
const osThreadAttr_t UserControl_0_attributes = {
  .name = "UserControl_0",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReadSensor_1 */
osThreadId_t ReadSensor_1Handle;
const osThreadAttr_t ReadSensor_1_attributes = {
  .name = "ReadSensor_1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for WriteScreen_2 */
osThreadId_t WriteScreen_2Handle;
const osThreadAttr_t WriteScreen_2_attributes = {
  .name = "WriteScreen_2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TECControl_3 */
osThreadId_t TECControl_3Handle;
const osThreadAttr_t TECControl_3_attributes = {
  .name = "TECControl_3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTask02(void *argument);  // ReadSensor_1
void StartTask03(void *argument);  // WriteScreen_2
void StartTask04(void *argument);  // TECControl_3
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartTask02(void *argument);
extern void StartTask03(void *argument);
extern void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  temperatureMutexHandle = osMutexNew(&temperatureMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UserControl_0 */
  UserControl_0Handle = osThreadNew(StartDefaultTask, NULL, &UserControl_0_attributes);

  /* creation of ReadSensor_1 */
  ReadSensor_1Handle = osThreadNew(StartTask02, NULL, &ReadSensor_1_attributes);

  /* creation of WriteScreen_2 */
  WriteScreen_2Handle = osThreadNew(StartTask03, NULL, &WriteScreen_2_attributes);

  /* creation of TECControl_3 */
  TECControl_3Handle = osThreadNew(StartTask04, NULL, &TECControl_3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  // UART接收缓冲区
  uint8_t rxBuffer[10];  // 增加缓冲区大小以处理更长的命令
  uint8_t commandBuffer[10];
  uint8_t commandLength = 0;
  
  // 启动UART接收
  HAL_UART_Receive_DMA(&huart2, rxBuffer, 1);
  
  /* Infinite loop */
  for(;;)
  {
    // 检查是否有接收到数据
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
      // 将接收到的数据添加到命令缓冲区
      if (rxBuffer[0] == '\r' || rxBuffer[0] == '\n') {
        // 处理命令结束符
        if (commandLength > 0) {
          commandBuffer[commandLength] = '\0';  // 字符串结束符
          
          // 处理命令
          if (commandLength == 2 && commandBuffer[0] == 'P' && commandBuffer[1] == '0') {
            // 关闭TEC
            tecEnabled = 0;
            TEC_Stop();
            // 发送确认信息
            HAL_UART_Transmit(&huart2, (uint8_t*)"TEC Disabled\r\n", 14, 100);
          }
          else if (commandLength == 2 && commandBuffer[0] == 'P' && commandBuffer[1] == '1') {
            // 启用TEC
            tecEnabled = 1;
            TEC_Start();
            // 发送确认信息
            HAL_UART_Transmit(&huart2, (uint8_t*)"TEC Enabled\r\n", 13, 100);
          }
          else if (commandLength == 3 && commandBuffer[0] == 'T' && 
                   commandBuffer[1] >= '0' && commandBuffer[1] <= '9' &&
                   commandBuffer[2] >= '0' && commandBuffer[2] <= '9') {
            // 设置目标温度
            uint8_t tempTens = commandBuffer[1] - '0';
            uint8_t tempOnes = commandBuffer[2] - '0';
            uint8_t tempValue = tempTens * 10 + tempOnes;
            
            // 检查温度范围
            if (tempValue <= 50) {
              // 获取互斥锁
              osStatus_t status = osMutexAcquire(temperatureMutexHandle, 100);
              if (status == osOK) {
                targetTemperature = (float)tempValue;
                // 释放互斥锁
                osMutexRelease(temperatureMutexHandle);
              }
              // 发送确认信息
              char response[30];
              snprintf(response, sizeof(response), "Target Temp Set: %d C\r\n", tempValue);
              HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            } else {
              // 温度超出范围
              HAL_UART_Transmit(&huart2, (uint8_t*)"Temp Out of Range (0-50)\r\n", 26, 100);
            }
          }
          else {
            // 无效命令
            HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid command\r\n", 17, 100);
          }
          
          // 重置命令长度
          commandLength = 0;
        }
      }
      else if (commandLength < 9) {
        // 添加字符到命令缓冲区
        commandBuffer[commandLength++] = rxBuffer[0];
      }
      else {
        // 缓冲区溢出，重置
        commandLength = 0;
      }
      
      // 重新启动接收
      HAL_UART_Receive_DMA(&huart2, rxBuffer, 1);
    }
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  ReadSensor_1任务函数
  * @param  argument: Not used
  * @retval None
  */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  float temperature;
  osStatus_t status;
  
  // 初始化DS18B20温度传感器
  DS18B20_Init();
  
  /* Infinite loop */
  for(;;)
  {
    // 读取温度传感器数据
    temperature = DS18B20_GetTemp();
    
    // 检查温度读取是否有效
    if (!isnan(temperature)) {
      // 获取互斥锁
      status = osMutexAcquire(temperatureMutexHandle, 100);
      if (status == osOK) {
        // 更新全局温度变量
        currentTemperature = temperature;
        // 释放互斥锁
        osMutexRelease(temperatureMutexHandle);
      }
    }
    
    // 每500ms读取一次温度
    osDelay(500);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  WriteScreen_2任务函数
  * @param  argument: Not used
  * @retval None
  */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  osStatus_t status;
  float localTemperature;
  uint8_t localTecPower;
  
  /* Infinite loop */
  for(;;)
  {
    // 获取互斥锁
    status = osMutexAcquire(temperatureMutexHandle, 100);
    if (status == osOK) {
      // 读取全局变量
      localTemperature = currentTemperature;
      localTecPower = tecPower;
      // 释放互斥锁
      osMutexRelease(temperatureMutexHandle);
      
      // 清除显示缓冲区
      CH1116_Clear();
      
      // 显示温度信息
      int tempIntPart = (int)localTemperature;
      int tempDecPart = (int)((localTemperature - tempIntPart) * 100);
      if (tempDecPart < 0) tempDecPart = -tempDecPart; // 处理负数情况
      snprintf(displayBuffer, sizeof(displayBuffer), "Temp: %d.%02d C", tempIntPart, tempDecPart);
      CH1116_PrintString(0, 0, displayBuffer);
      
      // 显示TEC功率信息
      snprintf(displayBuffer, sizeof(displayBuffer), "TEC Power: %d%%", localTecPower);
      CH1116_PrintString(2, 0, displayBuffer);
      
      // 更新显示
      CH1116_Display();
    }
    
    // 每1000ms更新一次显示
    osDelay(1000);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  TECControl_3任务函数
  * @param  argument: Not used
  * @retval None
  */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  osStatus_t status;
  float localTemperature;
  float pidOutput;
  uint8_t tecPowerValue;
  extern uint8_t tecEnabled; // 引用UserControl_0任务中的tecEnabled变量
  
  // 初始化PID控制器
  pidController.Kp = 2.0f;
  pidController.Ki = 0.5f;
  pidController.Kd = 0.1f;
  pidController.tau = 0.02f;
  pidController.limMin = 0.0f;
  pidController.limMax = 100.0f;
  pidController.limMinInt = -100.0f;
  pidController.limMaxInt = 100.0f;
  pidController.T = 0.5f; // 500ms采样周期
  PIDController_Init(&pidController);
  
  // 初始化TEC
  TEC_Init(&htim3, TIM_CHANNEL_2);
  TEC_Start();
  
  /* Infinite loop */
  for(;;)
  {
    // 检查TEC是否启用
    if (tecEnabled) {
      // 获取互斥锁
      status = osMutexAcquire(temperatureMutexHandle, 100);
      if (status == osOK) {
        // 读取全局变量
        localTemperature = currentTemperature;
        // 释放互斥锁
        osMutexRelease(temperatureMutexHandle);
        
        // 检查目标温度是否高于当前温度（TEC只能冷却）
        if (targetTemperature >= localTemperature) {
          // 目标温度高于或等于当前温度，TEC不工作
          TEC_SetPower(0);
          tecPowerValue = 0;
        } else {
          // 目标温度低于当前温度，使用PID控制器计算TEC功率
          pidOutput = PIDController_Update(&pidController, targetTemperature, localTemperature);
          
          // 限制输出范围并转换为整数
          pidOutput = fmaxf(0.0f, fminf(100.0f, pidOutput));
          tecPowerValue = (uint8_t)pidOutput;
          
          // 设置TEC功率
          TEC_SetPower(tecPowerValue);
        }
        
        // 更新全局TEC功率变量
        status = osMutexAcquire(temperatureMutexHandle, 100);
        if (status == osOK) {
          tecPower = tecPowerValue;
          osMutexRelease(temperatureMutexHandle);
        }
      }
    } else {
      // TEC未启用，确保功率为0
      TEC_SetPower(0);
      // 更新全局TEC功率变量
      status = osMutexAcquire(temperatureMutexHandle, 100);
      if (status == osOK) {
        tecPower = 0;
        osMutexRelease(temperatureMutexHandle);
      }
    }
    
    // 每500ms调整一次TEC功率
    osDelay(500);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE END Application */

