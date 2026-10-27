/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOE
#define M4_ADC_A_Pin GPIO_PIN_0
#define M4_ADC_A_GPIO_Port GPIOC
#define M4_ADC_B_Pin GPIO_PIN_1
#define M4_ADC_B_GPIO_Port GPIOC
#define M3_ADC_B_Pin GPIO_PIN_2
#define M3_ADC_B_GPIO_Port GPIOC
#define M3_ADC_A_Pin GPIO_PIN_3
#define M3_ADC_A_GPIO_Port GPIOC
#define M3_IN_A_Pin GPIO_PIN_0
#define M3_IN_A_GPIO_Port GPIOA
#define M3_IN_B_Pin GPIO_PIN_1
#define M3_IN_B_GPIO_Port GPIOA
#define M3_IN_C_Pin GPIO_PIN_2
#define M3_IN_C_GPIO_Port GPIOA
#define M34_IIC_SEL_Pin GPIO_PIN_5
#define M34_IIC_SEL_GPIO_Port GPIOA
#define M4_IN_C_Pin GPIO_PIN_6
#define M4_IN_C_GPIO_Port GPIOA
#define M4_IN_B_Pin GPIO_PIN_7
#define M4_IN_B_GPIO_Port GPIOA
#define M2_ADC_A_Pin GPIO_PIN_4
#define M2_ADC_A_GPIO_Port GPIOC
#define M2_ADC_B_Pin GPIO_PIN_5
#define M2_ADC_B_GPIO_Port GPIOC
#define M1_ADC_B_Pin GPIO_PIN_0
#define M1_ADC_B_GPIO_Port GPIOB
#define M1_ADC_A_Pin GPIO_PIN_1
#define M1_ADC_A_GPIO_Port GPIOB
#define M3_ENABLE_Pin GPIO_PIN_7
#define M3_ENABLE_GPIO_Port GPIOE
#define M4_ENABLE_Pin GPIO_PIN_8
#define M4_ENABLE_GPIO_Port GPIOE
#define MOTOR_CAL_Pin GPIO_PIN_9
#define MOTOR_CAL_GPIO_Port GPIOE
#define M1_ENABLE_Pin GPIO_PIN_10
#define M1_ENABLE_GPIO_Port GPIOE
#define M1_IN_C_Pin GPIO_PIN_10
#define M1_IN_C_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define M2_ENABLE_Pin GPIO_PIN_8
#define M2_ENABLE_GPIO_Port GPIOD
#define M2_IN_C_Pin GPIO_PIN_12
#define M2_IN_C_GPIO_Port GPIOD
#define M2_IN_B_Pin GPIO_PIN_13
#define M2_IN_B_GPIO_Port GPIOD
#define M2_IN_A_Pin GPIO_PIN_14
#define M2_IN_A_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOD
#define LCD_DC_Pin GPIO_PIN_6
#define LCD_DC_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOC
#define M4_IN_A_Pin GPIO_PIN_8
#define M4_IN_A_GPIO_Port GPIOC
#define M34_AS5600_SDA_Pin GPIO_PIN_9
#define M34_AS5600_SDA_GPIO_Port GPIOC
#define M34_AS5600_SCL_Pin GPIO_PIN_8
#define M34_AS5600_SCL_GPIO_Port GPIOA
#define M1_IN_B_Pin GPIO_PIN_15
#define M1_IN_B_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_10
#define IMU_SCK_GPIO_Port GPIOC
#define IMU_MISO_Pin GPIO_PIN_11
#define IMU_MISO_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_0
#define IMU_CS_GPIO_Port GPIOD
#define M12_IIC_SEL_Pin GPIO_PIN_1
#define M12_IIC_SEL_GPIO_Port GPIOD
#define M1_IN_A_Pin GPIO_PIN_3
#define M1_IN_A_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_4
#define KEY_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_5
#define IMU_MOSI_GPIO_Port GPIOB
#define M12_AS5600_SCL_Pin GPIO_PIN_6
#define M12_AS5600_SCL_GPIO_Port GPIOB
#define M12_AS5600_SDA_Pin GPIO_PIN_7
#define M12_AS5600_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
