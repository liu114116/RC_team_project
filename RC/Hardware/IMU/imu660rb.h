#ifndef _IMU660RB_H_
#define _IMU660RB_H_

#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "Fusion.h"
#include "usbd_cdc_if.h"

/* 根据实际接线修改 */
#define IMU_CS_PORT     GPIOD
#define IMU_CS_PIN      GPIO_PIN_0
#define IMU_SPI_HANDLE  hspi3

extern float acceleration_mg[3];
extern float angular_rate_mdps[3];

extern FusionAhrs  ahrs;
extern FusionEuler euler;

void IMU660RB_Init(void);
void Read_IMU660RB(void);
void IMU660RB_Print(void);

#endif
