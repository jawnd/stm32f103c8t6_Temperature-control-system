/*
 * DS18B20.h
 *
 *  Created on: Sep 12, 2025
 *      Author: admin
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "gpio.h"

#define DS18B20_DQ_H() HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET) //拉高电平
#define DS18B20_DQ_L() HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET) //拉低电平

//void DWT_Delay_Init(void);
//uint32_t DS18B20_Delay_us(uint32_t us);
void DS18B20_Init(void);
uint8_t DS18B20_Reset(void);
void DS18B20_WriteBit(uint8_t bit);
uint8_t DS18B20_ReadBit(void);
void DS18B20_WriteByte(uint8_t byte);
uint8_t DS18B20_ReadByte(void);
uint8_t DS18B20_ReadROM(uint8_t *rom);
float DS18B20_GetTemp(void);
uint8_t DS18B20_CheckCRC(uint8_t *data, uint8_t len);

#endif /* INC_DS18B20_H_ */
