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

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "lora.h"

uint8_t receive[80]; //receive data buffer
#define SPI1_DATA 0xD1
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SPI1_Configure_GPIOS(void);
void SPI1_Init(void);
void SPI_SendBuf(uint8_t *pBuf, uint32_t length);
uint8_t SPI1_SendRecv(uint8_t data);
static void MX_GPIO_Init(void);
void sendPacket(uint8_t data[], uint8_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t headerTo = 255;
uint8_t headerFrom = 255;
uint8_t headerID = 0;
uint8_t headerFlags = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  MX_GPIO_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  SPI1_Configure_GPIOS();
  SPI1_Init();
  LORA_INIT();
  //loraReceiveModeInit();

  uint8_t pack[] = "Snow";
  //loraTransmit(*pack, 4);
  readReg(RH_RF95_REG_00_FIFO);
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USER CODE BEGIN 4 */
void SPI1_Configure_GPIOS(void){
	/* Enable the peripheral clock of GPIOE */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;

	/* Select AF mode (10) on PE13 (CLK), PE14 (MISO), PE15 (MOSI) */
	GPIOE->MODER &= ~(GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15); // figure it out
	GPIOE->MODER |= (GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1); // figure it out

	/* AF5 for SPI1 signals in high E pins*/
	GPIOE->AFR[1] &= ~(0xF<<(4*(13-8)) | 0xF<<(4*(14-8)) | \
			0xF<<(4*(15-8)));
	GPIOE->AFR[1] |= (0x5<<(4*(13-8)) | 0x5<<(4*(14-8)) | \
			0x5<<(4*(15-8)));

}

void SPI1_Init(void){

	// Reset SPI peripheral
	RCC->APB2RSTR |=  RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	/* Enable SPI1 peripheral Clock */
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	/* Set control registers. Full duplex master mode, 64 clk divider, 8 data bits */
	/* MSB first */
	SPI1->CR1 = SPI_CR1_BIDIOE | SPI_CR1_BR_0 | SPI_CR1_BR_2 | SPI_CR1_MSTR | SPI_CR1_SSM;
	SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_SSOE; // | SPI_CR2_RXNEIE;
	SPI1->CR1 |= SPI_CR1_SPE;

	NVIC_SetPriority(SPI1_IRQn, 0); /* (4) */
	NVIC_EnableIRQ(SPI1_IRQn); /* (5) */
}
/* USER CODE END 4 */
// Send data buffer to SPI
// input:
//   SPIx - pointer to the SPI port
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: TX only mode
// note: function waits for transfer completion of the last byte
void SPI_SendBuf(uint8_t *pBuf, uint32_t length) {
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	do {
		while (!(SPI1->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		*((uint8_t *)&SPI1->DR) = *pBuf++;
	} while (--length);
	while (SPI1->SR & SPI_SR_BSY); // Wait for the transmission of the last byte
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

}

uint8_t SPI1_SendRecv(uint8_t data) {
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	SPI1->DR = data; // Send byte to SPI (TXE cleared)
	while (!(SPI1->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty
	while ((SPI1->SR & SPI_SR_BSY));
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

	return SPI1->DR; // Return received byte
}




//Function to burst write (primarily for FIFO)
void writeReg_Burst(uint8_t addr, uint8_t data[], uint8_t length)
{
	uint8_t reg = addr | 0x80;
	uint8_t val = 0;
	if (length >= 1)
	{
		while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
		HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
		SPI1->DR = reg; // Send byte to SPI (TXE cleared)
		while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
		for(int i = 0; i <= (length - 1); i++)
		{
			val = data[i];
			SPI1->DR = val; // Send byte to SPI (TXE cleared)
			while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
		}

		HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame
	}
}



//Function for reading from FIFO
void readFIFO(uint8_t buff[], uint16_t size)
{
	uint8_t reg = RH_RF95_REG_00_FIFO & ~0x80;
	while ((SPI1->SR & SPI_SR_BSY));
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	SPI1->DR = reg; // Send byte to SPI (TXE cleared)
	do {
		while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until TX buffer is empty
		*buff++ = *((uint8_t *)&SPI1->DR);
	} while (--size);
	while (SPI1->SR & SPI_SR_BSY); // Wait for the transmission of the last byte
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

}
/*
//Function for reading from a register
void receiveData()
{
	writeReg(RH_RF95_REG_01_OP_MODE, 0x01);
	writeReg(RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
	if (readReg(RH_RF95_REG_12_IRQ_FLAGS) == 0x00)
	{
		writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, readReg(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR)); //fifo addr ptr = fifo rx current addr
		uint8_t bytesLimit = readReg(RH_RF95_REG_13_RX_NB_BYTES);
		//HAL_Delay(10);
		readFIFO(receive, (uint16_t) bytesLimit);
		writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00);
	}
	writeReg(RH_RF95_REG_01_OP_MODE, 0x05);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x00);
}

void sendPacket(uint8_t data[], uint8_t size)
{
	writeReg(RH_RF95_REG_01_OP_MODE, 0x01); //STDBY
	writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00); //fifo addr pointer

	//set headers
	writeReg(RH_RF95_REG_00_FIFO, headerTo); //header TO
	writeReg(RH_RF95_REG_00_FIFO, headerFrom); //header FROM
	writeReg(RH_RF95_REG_00_FIFO, headerID); //header ID
	writeReg(RH_RF95_REG_00_FIFO, headerFlags); //header FLAGS

	//write message data to fifo
	writeReg_Burst(RH_RF95_REG_00_FIFO, data, size);

	//set payload length
	writeReg(RH_RF95_REG_22_PAYLOAD_LENGTH, size + RH_RF95_HEADER_LEN);

	//HAL_Delay(10); //delay some time

	writeReg(RH_RF95_REG_01_OP_MODE, 0x03); //TX Mode

	//while(readReg(RH_RF95_REG_12_IRQ_FLAGS) != 0x08); WRONG

	writeReg(RH_RF95_REG_01_OP_MODE, 0x01); //STDBY

	writeReg(RH_RF95_REG_12_IRQ_FLAGS, 0xFF); //clear txdone

	//set up for RX by default
	writeReg(RH_RF95_REG_01_OP_MODE, 0x05);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x00);
}

*/

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RST_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
