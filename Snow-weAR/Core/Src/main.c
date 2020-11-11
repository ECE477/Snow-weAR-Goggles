/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "GPS.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

uint8_t data[20] = {0};

void SystemClock_Config(void);
void Parse(void);

GPGGA_t GPGGA;

int main(void)
{

    HAL_Init();
    __HAL_RCC_USART2_CLK_ENABLE();

    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    //HAL_UART_Receive_DMA(&huart2, data, 10);
    //HAL_UART_Receive_IT(&huart2, data, 10);
    //uint8_t data1 = data[0];

    while (1)
    {
    	//uint8_t dataPoll[512] = {0};
      	//HAL_UART_Receive(&huart2, dataPoll, 512, 800);
      	//char* str=(char*)dataPoll;

    	char* str = "$GPGGA,224359.813,4600.234,N,86023.235,W";
    	int i = 1;
    	while(str[i-1] != ','){
    		i++;
    	}
      	GPGGA.UTC_Hour = 10*(str[i++]-48) + str[i++]-48;
      	GPGGA.UTC_Min = 10*(str[i++]-48) + str[i++]-48;
      	GPGGA.UTC_Sec = 10*(str[i++]-48) + str[i++]-48;
      	if(str[i] == '.') {
      		i++;
      	}
      	GPGGA.UTC_MicroSec = 100*(str[i++]-48) + 10*(str[i++]-48) + str[i++]-48;
      	if(str[i] == ',') {
      		i++;
      	}
      	while(str[i] != '.') {
      		GPGGA.Latitude = GPGGA.Latitude*10 + str[i++]-48;
      	}
      	i++;
      	while(str[i] != ',') {
      		GPGGA.LatitudeDecimal = GPGGA.LatitudeDecimal*10 + str[i++]-48;
      	}
      	i++;
      	GPGGA.NS_Indicator = str[i++];
      	i++;
      	while(str[i] != '.') {
      		GPGGA.Longitude = GPGGA.Longitude*10 + str[i++]-48;
      	}
      	i++;
      	while(str[i] != ','){
      		GPGGA.LongitudeDecimal = GPGGA.LongitudeDecimal*10 + str[i++]-48;
      	}
      	i++;
      	GPGGA.EW_Indicator = str[i++];

      	//long oops = strtoul(str, 10, 10);
		//sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS.GPGGA.UTC_Hour,&GPS.GPGGA.UTC_Min,&GPS.GPGGA.UTC_Sec,&GPS.GPGGA.UTC_MicroSec,&GPS.GPGGA.Latitude,&GPS.GPGGA.NS_Indicator,&GPS.GPGGA.Longitude,&GPS.GPGGA.EW_Indicator,&GPS.GPGGA.PositionFixIndicator,&GPS.GPGGA.SatellitesUsed,&GPS.GPGGA.HDOP,&GPS.GPGGA.MSL_Altitude,&GPS.GPGGA.MSL_Units,&GPS.GPGGA.AgeofDiffCorr,GPS.GPGGA.DiffRefStationID,GPS.GPGGA.CheckSum);
		HAL_Delay(300);
    }
}

void Parse(void){

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	HAL_UART_Receive_IT(&huart2, data, 10);
	//HAL_UART_Receive_DMA(&huart2, data, 10);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
