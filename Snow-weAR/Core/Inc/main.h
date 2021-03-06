/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include "stm32l4xx_hal.h"
#include "LoRa.h"

void Error_Handler(void);

#define LoRaDev // just initialize GPS, oLED, LoRa


#define BB_GPOUT_Pin GPIO_PIN_0
#define BB_GPOUT_GPIO_Port GPIOH
#define BB_CE_Pin GPIO_PIN_1
#define BB_CE_GPIO_Port GPIOH
#define BB_SYSOFF_Pin GPIO_PIN_0
#define BB_SYSOFF_GPIO_Port GPIOA
#define session_btn_Pin GPIO_PIN_1
#define session_btn_GPIO_Port GPIOA
#define session_btn_EXTI_IRQn EXTI1_IRQn
#define radio_btn_Pin GPIO_PIN_1
#define radio_btn_GPIO_Port GPIOA
#define GPS_Rx_Pin GPIO_PIN_3
#define GPS_Rx_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define IMU_ADDR_Pin GPIO_PIN_12
#define IMU_ADDR_GPIO_Port GPIOE
#define IMU_INT_Pin GPIO_PIN_13
#define IMU_INT_GPIO_Port GPIOE
#define IMU_nRST_Pin GPIO_PIN_14
#define IMU_nRST_GPIO_Port GPIOE
#define IMU_BL_IND_Pin GPIO_PIN_15
#define IMU_BL_IND_GPIO_Port GPIOE
#define GPSReg_ERR_Pin GPIO_PIN_7
#define GPSReg_ERR_GPIO_Port GPIOC
#define GPSReg_Pwr_Pin GPIO_PIN_8
#define GPSReg_Pwr_GPIO_Port GPIOC

#ifdef LoRaDev

#define DEV 1
#define RXDone_GPIO_Port 		GPIOA
#define RXDone_Pin 				GPIO_PIN_2
#define RXDone_EXTI_IRQn 		EXTI2_IRQn
#define LORA_GPIO_Port			GPIOE
#define LORA_NSS_GPIO_Port		GPIOE
#define LORA_NSS_Pin			GPIO_PIN_12
#define LORA_RST_GPIO_Port		GPIOE
#define LORA_RST_Pin			GPIO_PIN_10
#define LORA_SPI_Port			GPIOE
#define LORA_SCLK_Pin			GPIO_PIN_13
#define LORA_MISO_Pin			GPIO_PIN_14
#define LORA_MOSI_Pin			GPIO_PIN_15
#define LORA_EXTI_IRQn			EXTI15_10_IRQn
#define REG_FIFO				0x00


#else

#define DEV 0
#define CS_LoRa_Pin GPIO_PIN_4
#define CS_LoRa_GPIO_Port GPIOA
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
