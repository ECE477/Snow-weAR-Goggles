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

#define LINEMAX 200

uint8_t data[1000] = {0};
uint8_t byte;
int idx = 0;
int count = 0;
int numBytes = 0;

volatile char line_buffer[LINEMAX + 1];
volatile int line_valid = 0;

void SystemClock_Config(void);
void GPS_Parse(void);
void USART2_IRQHandler(void);

int main(void)
{
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    while (1)
    {
    	if(line_valid == 1) {
    		GPS_Parse();
    	}
    }
}
void USART2_IRQHandler(void) {
	//byte = USART2->RDR;
	//char rx = (char)(USART2->RDR & 0xFF);
	//GPS.rxTmp = rx;

	static char rx_buffer[LINEMAX];
	static int rx_index = 0;

	if(USART2->ISR & USART_ISR_RXNE) {
		char rx = (char)(USART2->RDR & 0xFF);
		data[idx++] = rx;

		if(rx == '\n') {
			idx = 0;
			numBytes++;
			line_valid = 1;
		}
/*
		if((rx == '\r') || (rx == '\n')) {
			if(rx_index != 0) {
				memcpy((void*)line_buffer, rx_buffer, rx_index);
				line_buffer[rx_index] = 0;
				line_valid = 1;
				rx_index = 0;
			}
		}
		else {
			if((rx == '$') || (rx_index == LINEMAX)) {
				rx_index = 0;
				numBytes++;
			}
			rx_buffer[rx_index++] = rx;
		}
		*/
	}

	/*
	if((GPS.rxBuffer[0] == '$' || (idx == 0 && rx == '$')) && idx < 512) {
		data[idx] = rx;
		GPS.rxBuffer[idx++] = rx;
		GPS.rxIndex++;
		count++;
	}
	if(idx > 511) {
		idx = 0;
		GPS_Parse();
	}*/
}

void GPS_Parse(void){

	char* str = (char*) data;

	//$GPGGA,222922.000,4025.9453,N,08654.4422,W,1,06,1.27,188.5,M,-33.8,M,,*51\r\n
	int j=0;
	while((str[j] != '$' || str[j+1] != 'G' || str[j+2] != 'P' || str[j+3] != 'G' || str[j+4] != 'G' || str[j+5] != 'A') && (j+1) < strlen(str)) {
		j++;
	}
	int i = j+7;
	while(str[i-1] != ',') {
		i++;
	}
	GPS.GPGGA.UTC_Hour = 10*(str[i++]-48) + str[i++]-53;
	GPS.GPGGA.UTC_Min = 10*(str[i++]-48) + str[i++]-48;
	GPS.GPGGA.UTC_Sec = 10*(str[i++]-48) + str[i++]-48;
	i++;
	GPS.GPGGA.UTC_MicroSec = 100*(str[i++]-48) + 10*(str[i++]-48) + str[i++]-48;
	i++;
	while(str[i] != '.') {
		GPS.GPGGA.Latitude = GPS.GPGGA.Latitude*10 + str[i++]-48;
	}
	i++;
	int divFactor = 1;
	while(str[i] != ',') {
		GPS.GPGGA.LatitudeDecimal = GPS.GPGGA.LatitudeDecimal*10 + str[i++]-48;
		divFactor *= 10;
	}
	GPS.GPGGA.Latitude = GPS.GPGGA.Latitude + GPS.GPGGA.LatitudeDecimal / divFactor;
	i++;
	GPS.GPGGA.NS_Indicator = str[i++];
	i++;
	while(str[i] != '.') {
		GPS.GPGGA.Longitude = GPS.GPGGA.Longitude*10 + str[i++]-48;
	}
	i++;
	divFactor = 1;
	while(str[i] != ',') {
		GPS.GPGGA.LongitudeDecimal = GPS.GPGGA.LongitudeDecimal*10 + str[i++]-48;
		divFactor *= 10;
	}
	i++;
	GPS.GPGGA.EW_Indicator = str[i++];
	GPS.GPGGA.LatitudeDecimal = convertDegMinToDecDeg(GPS.GPGGA.Latitude);
	GPS.GPGGA.LongitudeDecimal = convertDegMinToDecDeg(GPS.GPGGA.Longitude);

	int comma = 0;
	while(comma < 4) {
		if(str[i++] == ',') {
			comma++;
		}
	}
	while(str[i] != '.') {
		GPS.GPGGA.MSL_Altitude = GPS.GPGGA.MSL_Altitude*10 + str[i++]-48;
	}
	int altDecimal = 0;
	divFactor = 1;
	while(str[i] != ',') {
		altDecimal = altDecimal*10 + str[i++]-48;
		divFactor *= 10;
	}
	GPS.GPGGA.MSL_Altitude = GPS.GPGGA.MSL_Altitude + altDecimal / divFactor;

	line_valid = 0;
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
