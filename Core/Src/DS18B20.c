/*
 * DS18B20.c
 *
 *  Created on: Sep 12, 2025
 *      Author: admin
 */
#include "main.h"
#include "gpio.h"
#include "DS18B20.h"
#include "string.h"



// 设置为输出模式（推挽或开漏）
static void DS18B20_SetOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_Pin; //指向结构 GPIO_InitTypeDef 的指针 DS18B20_Pin为cubemx中设置的自定义引脚名称
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出（推荐）
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

// 设置为输入模式（高阻态，靠上拉电阻拉高）
static void DS18B20_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // 外部已有 4.7kΩ 上拉
    HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

// 初始化（在 main 中调用一次）
void DS18B20_Init(void)
{
    DS18B20_SetOutput();
    DS18B20_DQ_H();
}

// 1-Wire 复位脉冲
uint8_t DS18B20_Reset(void)
{
    uint8_t presence = 0;

    DS18B20_SetOutput();
    DS18B20_DQ_L();              // 拉低总线
    DS18B20_Delay_us(480);       // 保持 480us
    DS18B20_SetInput();          // 释放总线（转为输入）
    DS18B20_Delay_us(70);        // 等待 60~70us

    if (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin) == GPIO_PIN_RESET)
        presence = 1;            // 检测到应答脉冲（低电平）

    DS18B20_Delay_us(410);       // 等待复位时序结束（总 > 480us）
    return presence;
}

// 写一位
void DS18B20_WriteBit(uint8_t bit)
{
    DS18B20_SetOutput();
    DS18B20_DQ_L();              // 拉低开始写时隙
    DS18B20_Delay_us(1);         // >1us

    if (bit)
        DS18B20_DQ_H();          // 写1：释放总线
    // 如果写0，保持低电平

    DS18B20_Delay_us(60);        // 整个时隙 60~120us
    DS18B20_SetInput();          // 释放总线（即使写0，也应在时隙结束后释放）
}

// 读一位
uint8_t DS18B20_ReadBit(void)
{
    uint8_t bit = 0;

    DS18B20_SetOutput();
    DS18B20_DQ_L();              // 拉低开始读时隙
    DS18B20_Delay_us(1);         // >1us
    DS18B20_SetInput();          // 释放总线，准备读取

    DS18B20_Delay_us(10);        // 等待 15us 内读取
    if (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin) == GPIO_PIN_SET)
        bit = 1;

    DS18B20_Delay_us(50);        // 完成整个读时隙（>60us）
    return bit;
}

// 写一字节
void DS18B20_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

// 读一字节
uint8_t DS18B20_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        byte >>= 1;
        if (DS18B20_ReadBit())
            byte |= 0x80;
    }
    return byte;
}

// 读取 ROM（64位 = 8字节）
uint8_t DS18B20_ReadROM(uint8_t *rom)
{
    if (!DS18B20_Reset()) return 0;

    DS18B20_WriteByte(0x33);     // Read ROM

    for (int i = 0; i < 8; i++)
    {
        rom[i] = DS18B20_ReadByte();
    }

    return DS18B20_CheckCRC(rom, 8);
}

// CRC8 校验（用于 ROM 和 Scratchpad）
uint8_t DS18B20_CheckCRC(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;  // 多项式 0x8C (X8 + X5 + X4 + 1)
            else
                crc >>= 1;
        }
    }
    return (crc == 0);
}

// 获取温度（跳过 ROM，适用于单个传感器）
float DS18B20_GetTemp(void)
{
    uint8_t data[9];
    int16_t raw_temp;

    if (!DS18B20_Reset()) return -999.9f;

    DS18B20_WriteByte(0xCC);     // Skip ROM
    DS18B20_WriteByte(0x44);     // Convert T
    HAL_Delay(750);              // 等待转换完成（最长 750ms）

    if (!DS18B20_Reset()) return -999.9f;

    DS18B20_WriteByte(0xCC);     // Skip ROM
    DS18B20_WriteByte(0xBE);     // Read Scratchpad

    for (int i = 0; i < 9; i++)
    {
        data[i] = DS18B20_ReadByte();
    }

    if (!DS18B20_CheckCRC(data, 9))
        return -999.9f;          // CRC 校验失败

    raw_temp = (data[1] << 8) | data[0];
    return (float)raw_temp / 16.0f;
}
