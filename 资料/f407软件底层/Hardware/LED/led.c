#include "led.h"

void led_set(uint8_t led,uint8_t state){
	if(led == 1){
		if(state == 1){
			HAL_GPIO_WritePin(LED_R_PORT,LED_R_PIN,GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(LED_R_PORT,LED_R_PIN,GPIO_PIN_SET);
		}
	}
	if(led == 2){
		if(state == 1){
			HAL_GPIO_WritePin(LED_G_PORT,LED_G_PIN,GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(LED_G_PORT,LED_G_PIN,GPIO_PIN_SET);
		}
	}
	if(led == 3){
		if(state == 1){
			HAL_GPIO_WritePin(LED_B_PORT,LED_B_PIN,GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(LED_B_PORT,LED_B_PIN,GPIO_PIN_SET);
		}
	}
}

