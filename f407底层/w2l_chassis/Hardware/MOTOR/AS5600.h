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

/*
 * 启动一次非阻塞读取（I2C + DMA）。
 * 返回 1：已启动；返回 0：上一次还没完成，或参数无效。
 * 完成后由 HAL_I2C_MemRxCpltCallback 存结果，用下面两个函数取。
 */
uint8_t Encoder_StartReadDMA(uint8_t id);
/*
 * 返回 1：读取成功，*raw 有效，范围 0~4095。
 * 返回 0：读取失败或参数无效，输出变量不修改。
 */
uint8_t Encoder_ReadRaw(uint8_t id, uint16_t *raw);
/*
 * 返回 1：读取成功，*angle 有效，单位 rad，范围 [0, 2π)。
 * 返回 0：读取失败或参数无效，输出变量不修改。
 */
uint8_t Encoder_ReadAngle(uint8_t id, float *angle);

#endif
