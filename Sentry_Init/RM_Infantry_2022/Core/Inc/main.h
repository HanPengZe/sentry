/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define Vision_Pin GPIO_PIN_14
#define Vision_GPIO_Port GPIOG
#define IMU_Pin GPIO_PIN_4
#define IMU_GPIO_Port GPIOB
#define IMUB3_Pin GPIO_PIN_3
#define IMUB3_GPIO_Port GPIOB
#define Debug_Pin GPIO_PIN_7
#define Debug_GPIO_Port GPIOB
#define DR16_Pin GPIO_PIN_11
#define DR16_GPIO_Port GPIOC
#define DR16C10_Pin GPIO_PIN_10
#define DR16C10_GPIO_Port GPIOC
#define VisionG9_Pin GPIO_PIN_9
#define VisionG9_GPIO_Port GPIOG
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOF
#define DebugA9_Pin GPIO_PIN_9
#define DebugA9_GPIO_Port GPIOA
#define Laser_Pin GPIO_PIN_8
#define Laser_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_1
#define OLED_RST_GPIO_Port GPIOF
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define IMU_heat_Pin GPIO_PIN_6
#define IMU_heat_GPIO_Port GPIOF
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define Bat_Voltage_Pin GPIO_PIN_10
#define Bat_Voltage_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define Bzzer_Pin GPIO_PIN_14
#define Bzzer_GPIO_Port GPIOD
#define Key_Pin GPIO_PIN_0
#define Key_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define magazine_pwm_Pin GPIO_PIN_9
#define magazine_pwm_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define OLED_SCK_Pin GPIO_PIN_13
#define OLED_SCK_GPIO_Port GPIOB
#define IMUA7_Pin GPIO_PIN_7
#define IMUA7_GPIO_Port GPIOA
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
#define OLED_MOSI_Pin GPIO_PIN_15
#define OLED_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
