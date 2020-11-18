/*
 * lora.c
 *
 *  Created on: Nov 12, 2020
 *      Author: carrie
 */
#include "lora.h"

// BIG MASTER FUNCTIONS:
void loraTransmit(uint8_t *buffer, uint8_t len){

	// enter standby mode

	// Start SPI transaction, send address
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	loraWriteFIFO(buffer, len);

	// End SPI transaction
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

	// enter transmit mode

	// watch txdone irq

	// should be transmit mode again
}

void loraWriteFIFO(uint8_t *buffer, uint8_t len){
	// set fifoptraddr to fifotxptrbase
	uint8_t txPtrBase = readReg(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR);
	writeReg(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR, txPtrBase);

	// Write Payload Length bytes to the FIFO (RegFifo)
	uint8_t reg = REG_FIFO | 0x80;
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	// while loop to send entire buffer
	uint8_t value = *((uint8_t *)buffer);
	buffer += sizeof(uint8_t);
	len--;
	SPI1->DR = reg | value << 8; // Send byte to SPI (TXE cleared)
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	while(len > 0){
		uint8_t val0 = *((uint8_t *)buffer);
		buffer += sizeof(uint8_t);
		len--;
		uint8_t val1 = 0;
		if(len > 0){
			val1 = *((uint8_t *)buffer);
			buffer += sizeof(uint8_t);
			len--;
		}
		SPI1->DR = val0 | val1 << 8; // Send byte to SPI (TXE cleared)
		while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	}

	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame

	return;
}

void loraReceiveModeInit(void){
	// make sure mode is standby or sleep
	loraStandbyMode();

	writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);

}

void loraStandbyMode(void){
	uint8_t mode = readReg(RH_RF95_REG_01_OP_MODE) & 0x3;
	if((mode != RH_RF95_MODE_STDBY)){
		writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	}
}

void loraReadFIFO(uint8_t *buf){
	// check for payload crc error. Make sure rxtimeout isn't set

	// read regrxnbbytes to find the num of bytes recieved

	// set regfifoaddrptr to regfiforx current addr.

	// regfifo regrxnbbytes number of times
	/*static void read_fifo(lora_sx1276 *lora, uint8_t *buffer, uint8_t len, uint8_t mode)
{
  uint8_t address = REG_FIFO;

  // Start SPI transaction, send address
  HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_RESET);
  uint32_t res1 = HAL_SPI_Transmit(lora->spi, &address, 1, lora->spi_timeout);
  uint32_t res2;
  if (mode == TRANSFER_MODE_DMA) {
    res2 = HAL_SPI_Receive_DMA(lora->spi, buffer, len);
    // Do not end SPI here - must be done externally when DMA done
  } else {
    res2 = HAL_SPI_Receive(lora->spi, buffer, len, lora->spi_timeout);
    // End SPI transaction
    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_SET);
  }

  if (res1 != HAL_OK || res2 != HAL_OK) {
    DEBUGF("SPI receive/transmit failed");
  }
}*/
    /*while (hspi->RxXferCount > 0U)
    {
      if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
      {
        (* (uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DR;
        hspi->pRxBuffPtr += sizeof(uint8_t);
        hspi->RxXferCount--;
      }*/
}

//Function for reading from a register
uint8_t readReg(uint8_t addr){
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	while (!(SPI1->SR & SPI_SR_TXE)); // Wait while receive buffer is empty
	SPI1->DR = addr; // Send byte to SPI (TXE cleared)
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	//while ((SPI1->SR & SPI_SR_BSY));
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

	return SPI1->DR; // Return received byte
}

void writeReg(uint8_t addr, uint8_t value){
	uint8_t reg = addr | 0x80;
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	SPI1->DR = reg | value << 8; // Send byte to SPI (TXE cleared)
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame

}

void LORA_INIT(void){
	//reset LoRa
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	//initialization
	//writeReg(RH_RF95_REG_00_FIFO, 0x00);
	writeReg(RH_RF95_REG_01_OP_MODE, 0x80); //long range mode
	//readReg(RH_RF95_REG_01_OP_MODE);
	writeReg(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0x00); //tx base addr to 0
	writeReg(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0x00); //rx base addr to 0
	writeReg(RH_RF95_REG_1D_MODEM_CONFIG1, 0x72); //coding rate and modem config
	writeReg(RH_RF95_REG_1E_MODEM_CONFIG2, 0x74); //rxpayloadcrc and spreading factor
	writeReg(RH_RF95_REG_26_MODEM_CONFIG3, 0x04); //LNA gain
	writeReg(RH_RF95_REG_20_PREAMBLE_MSB, 0x00); //preamble MSB
	writeReg(RH_RF95_REG_21_PREAMBLE_LSB, 0x08); //premamble LSB
	writeReg(RH_RF95_REG_06_FRF_MSB, 0x6C); //freq msb
	writeReg(RH_RF95_REG_07_FRF_MID, 0x80); //freq mid
	writeReg(RH_RF95_REG_08_FRF_LSB, 0x00); //freq lsb
	writeReg(RH_RF95_REG_4D_PA_DAC, 0x04); //padac
	writeReg(RH_RF95_REG_09_PA_CONFIG, 0x88); //output power and PA_BOOST

	//set frequency to 915MHz
	writeReg(RH_RF95_REG_06_FRF_MSB, 0xE4); //freq msb CHANGE BACK TOE4
	writeReg(RH_RF95_REG_07_FRF_MID, 0xC0); //freq mid
	writeReg(RH_RF95_REG_08_FRF_LSB, 0x00); //freq lsb

	//set power
	writeReg(RH_RF95_REG_4D_PA_DAC, 0x07); //padac
	writeReg(RH_RF95_REG_09_PA_CONFIG, 0x8F); //output power and PA_BOOST

	//set up for RX by default
	writeReg(RH_RF95_REG_01_OP_MODE, 0x05);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x00);
}
