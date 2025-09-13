/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ch1116.c
  * @brief   该文件提供CH1116 OLED显示屏驱动的代码
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

/* Includes ------------------------------------------------------------------*/
#include "ch1116.h"

/* USER CODE BEGIN 0 */
/* Buffer to hold display data */
static uint8_t CH1116_Buffer[CH1116_WIDTH * CH1116_PAGES];

/* Cursor position */
static uint8_t CH1116_CurrentPage = 0;
static uint8_t CH1116_CurrentColumn = 0;

/* DMA transmission status */
static volatile uint8_t dma_transmission_complete = 1;

/**
  * @brief  向CH1116写入命令
  * @param  command: 要写入的命令
  * @retval 无
  */
void CH1116_WriteCommand(uint8_t command)
{
    dma_transmission_complete = 0;
    uint8_t control_byte = 0x00; // Control byte for command
    HAL_I2C_Mem_Write_DMA(&hi2c2, CH1116_I2C_ADDR << 1, control_byte, 1, &command, 1);
    
    // 等待传输完成
    while (!dma_transmission_complete)
    {
        // 可以添加超时处理
    }
}

/**
  * @brief  向CH1116写入数据
  * @param  data: 要写入的数据
  * @param  size: 数据大小
  * @retval 无
  */
void CH1116_WriteData(uint8_t *data, uint16_t size)
{
    dma_transmission_complete = 0;
    uint8_t control_byte = 0x40; // Control byte for data
    HAL_I2C_Mem_Write_DMA(&hi2c2, CH1116_I2C_ADDR << 1, control_byte, 1, data, size);
    
    // 等待传输完成
    while (!dma_transmission_complete)
    {
        // 可以添加超时处理
    }
}

/**
  * @brief  DMA传输完成回调函数
  * @retval 无
  */
void CH1116_DMA_TransmitComplete(void)
{
    dma_transmission_complete = 1;
}

/**
  * @brief  检查传输是否完成
  * @retval 1: 传输完成, 0: 传输未完成
  */
uint8_t CH1116_IsTransmissionComplete(void)
{
    return dma_transmission_complete;
}

/* USER CODE END 0 */

/**
  * @brief  初始化CH1116 OLED显示屏
  * @retval 无
  */
void CH1116_Init(void)
{
    /* USER CODE BEGIN CH1116_Init */
    // Wait for power stabilization
    HAL_Delay(100);
    
    // Initialization sequence for CH1116
    CH1116_WriteCommand(CH1116_CMD_DISPLAY_OFF);                 // 关闭显示
    CH1116_WriteCommand(CH1116_CMD_SET_MUX_RATIO);               // 设置多路复用比率
    CH1116_WriteCommand(0x3F);                                   // 1/64占空比
    CH1116_WriteCommand(CH1116_CMD_SET_DISPLAY_OFFSET);          // 设置显示偏移
    CH1116_WriteCommand(0x00);                                   // 无偏移
    CH1116_WriteCommand(CH1116_CMD_SET_START_LINE | 0x00);       // 设置起始行
    CH1116_WriteCommand(CH1116_CMD_SET_SEGMENT_REMAP);           // 设置段重映射
    CH1116_WriteCommand(CH1116_CMD_SET_COM_SCAN_DIR);            // 设置COM输出扫描方向
    CH1116_WriteCommand(CH1116_CMD_SET_COM_PINS);                // 设置COM引脚硬件配置
    CH1116_WriteCommand(0x12);                                   // 替代COM引脚配置
    CH1116_WriteCommand(CH1116_CMD_SET_CONTRAST);                // 设置对比度控制
    CH1116_WriteCommand(0xCF);                                   // 对比度值
    CH1116_WriteCommand(CH1116_CMD_SET_PRECHARGE);               // 设置预充电周期
    CH1116_WriteCommand(0xF1);                                   // 预充电周期
    CH1116_WriteCommand(CH1116_CMD_SET_VCOMH_DESELECT);          // 设置VCOMH去选中电压等级
    CH1116_WriteCommand(0x40);                                   // VCOMH去选中电压等级
    CH1116_WriteCommand(CH1116_CMD_CHARGE_PUMP);                 // 启用电荷泵调节器
    CH1116_WriteCommand(0x14);                                   // 电荷泵开启
    CH1116_WriteCommand(CH1116_CMD_DISPLAY_ON);                  // 开启显示
    
    // Clear display
    CH1116_Clear();
    CH1116_Display();
    /* USER CODE END CH1116_Init */
}

/**
  * @brief  清除CH1116显示缓冲区
  * @retval 无
  */
void CH1116_Clear(void)
{
    /* USER CODE BEGIN CH1116_Clear */
    for (uint16_t i = 0; i < sizeof(CH1116_Buffer); i++)
    {
        CH1116_Buffer[i] = 0x00;
    }
    /* USER CODE END CH1116_Clear */
}

/**
  * @brief  用缓冲区数据更新CH1116显示屏
  * @retval 无
  */
void CH1116_Display(void)
{
    /* USER CODE BEGIN CH1116_Display */
    for (uint8_t page = 0; page < CH1116_PAGES; page++)
    {
        CH1116_SetCursor(page, 0);
        CH1116_WriteData(&CH1116_Buffer[CH1116_WIDTH * page], CH1116_WIDTH);
    }
    /* USER CODE END CH1116_Display */
}

/**
  * @brief  Set cursor position on CH1116 display
  * @param  page: Page address (0-7)
  * @param  column: Column address (0-127)
  * @retval None
  */
void CH1116_SetCursor(uint8_t page, uint8_t column)
{
    /* USER CODE BEGIN CH1116_SetCursor */
    CH1116_CurrentPage = page;
    CH1116_CurrentColumn = column;
    
    // Set page and column addresses
    CH1116_WriteCommand(CH1116_CMD_SET_PAGE | page);
    CH1116_WriteCommand(CH1116_CMD_SET_LOW_COLUMN | (column & 0x0F));
    CH1116_WriteCommand(CH1116_CMD_SET_HIGH_COLUMN | ((column >> 4) & 0x0F));
    /* USER CODE END CH1116_SetCursor */
}

/**
  * @brief  在指定位置绘制像素点
  * @param  x: X坐标 (0-127)
  * @param  y: Y坐标 (0-63)
  * @param  color: 像素颜色 (0 = 黑色, 1 = 白色)
  * @retval 无
  */
void CH1116_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    /* USER CODE BEGIN CH1116_DrawPixel */
    if (x >= CH1116_WIDTH || y >= CH1116_HEIGHT)
        return;
    
    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    
    if (color)
    {
        CH1116_Buffer[x + (page * CH1116_WIDTH)] |= (1 << bit);
    }
    else
    {
        CH1116_Buffer[x + (page * CH1116_WIDTH)] &= ~(1 << bit);
    }
    /* USER CODE END CH1116_DrawPixel */
}

/**
  * @brief  用指定模式填充整个显示缓冲区
  * @param  color: 填充颜色 (0 = 黑色, 1 = 白色)
  * @retval 无
  */
void CH1116_Fill(uint8_t color)
{
    /* USER CODE BEGIN CH1116_Fill */
    uint8_t fill_byte = color ? 0xFF : 0x00;
    for (uint16_t i = 0; i < sizeof(CH1116_Buffer); i++)
    {
        CH1116_Buffer[i] = fill_byte;
    }
    /* USER CODE END CH1116_Fill */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */