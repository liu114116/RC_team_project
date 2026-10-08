#ifndef _AS5600_H_
#define _AS5600_H_
#include "main.h"
#include "i2c.h" 

/*
编码器分配设置
	____________________
				 FPC接口
	|	left        right	|
	|	I2C 3     	I2C	1	|
	|	I2C 4     	I2C 2	|
	____________________
*/

#define AS5600_Address       0x36   
#define RAW_Angle_Hi         0x0C
#define I2C_TIMEOUT          2

#define AS5600_12_SEL_PORT GPIOD
#define AS5600_12_SEL_PIN GPIO_PIN_1
#define AS5600_34_SEL_PORT GPIOA
#define AS5600_34_SEL_PIN GPIO_PIN_5

uint16_t AS5600_ReadHardware(I2C_HandleTypeDef *hi2c);

uint16_t Encoder_ReadRaw(uint8_t id);



#endif
