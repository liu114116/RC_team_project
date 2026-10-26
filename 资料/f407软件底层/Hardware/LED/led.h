#ifndef __LED_H
#define __LED_H

#include "main.h"

#define LED_R_PORT 	GPIOE
#define LED_R_PIN 	GPIO_PIN_2
#define LED_G_PORT 	GPIOE
#define LED_G_PIN 	GPIO_PIN_3
#define LED_B_PORT 	GPIOE
#define LED_B_PIN 	GPIO_PIN_4

void led_set(uint8_t led,uint8_t state);

#endif
