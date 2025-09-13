/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ch1116.h
  * @brief   该文件包含ch1116.c文件的所有函数原型
  ******************************************************************************
  * @attention
  *
  * 版权所有 (c) 2025 STMicroelectronics。
  * 保留所有权利。
  *
  * 该软件根据可以在LICENSE文件中找到的条款进行许可
  * 在此软件组件的根目录中。
  * 如果没有随此软件一起提供的LICENSE文件，则按原样提供。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CH1116_H__
#define __CH1116_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
#define CH1116_I2C_ADDR        0x3C  // 7-bit I2C address
#define CH1116_WIDTH           128
#define CH1116_HEIGHT          64
#define CH1116_PAGES           (CH1116_HEIGHT / 8)

/* CH1116命令定义 */
#define CH1116_CMD_DISPLAY_OFF          0xAE  // 关闭显示
#define CH1116_CMD_DISPLAY_ON           0xAF  // 开启显示
#define CH1116_CMD_SET_LOW_COLUMN       0x00  // 设置列地址低4位
#define CH1116_CMD_SET_HIGH_COLUMN      0x10  // 设置列地址高4位
#define CH1116_CMD_SET_PAGE             0xB0  // 设置页地址
#define CH1116_CMD_SET_START_LINE       0x40  // 设置起始行
#define CH1116_CMD_SET_CONTRAST         0x81  // 设置对比度
#define CH1116_CMD_SET_SEGMENT_REMAP    0xA1  // 设置段重映射
#define CH1116_CMD_SET_COM_SCAN_DIR     0xC8  // 设置COM扫描方向
#define CH1116_CMD_SET_MUX_RATIO        0xA8  // 设置多路复用比率
#define CH1116_CMD_SET_DISPLAY_OFFSET   0xD3  // 设置显示偏移
#define CH1116_CMD_SET_CLOCK_DIV        0xD5  // 设置时钟分频
#define CH1116_CMD_SET_PRECHARGE        0xD9  // 设置预充电周期
#define CH1116_CMD_SET_COM_PINS         0xDA  // 设置COM引脚配置
#define CH1116_CMD_SET_VCOMH_DESELECT   0xDB  // 设置VCOMH去选中电压
#define CH1116_CMD_CHARGE_PUMP          0x8D  // 设置电荷泵
#define CH1116_CMD_SET_MEMORY_ADDR_MODE 0x20  // 设置内存地址模式
#define CH1116_CMD_SET_COLUMN_ADDR      0x21  // 设置列地址
#define CH1116_CMD_SET_PAGE_ADDR        0x22  // 设置页地址

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void CH1116_Init(void);
void CH1116_WriteCommand(uint8_t command);
void CH1116_WriteData(uint8_t *data, uint16_t size);
void CH1116_Clear(void);
void CH1116_Display(void);
void CH1116_SetCursor(uint8_t page, uint8_t column);
void CH1116_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void CH1116_Fill(uint8_t color);

/* DMA related functions */
void CH1116_DMA_TransmitComplete(void);
uint8_t CH1116_IsTransmissionComplete(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __CH1116_H__ */