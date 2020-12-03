/*
 * LoRa.c
 *
 *  Created on: Nov 21, 2020
 *      Author: carrie
 */
#include "LoRa.h"
#include "main.h"

uint8_t readReg(uint8_t addr){
	// Clear Data register
	uint16_t dummy;
	while ((SPI1->SR & SPI_SR_RXNE)){
		dummy = SPI1->DR;
	}

	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while bsy
	SPI1->DR = (uint16_t)addr; // Send byte to SPI (TXE cleared)
	while (!(SPI1->SR & SPI_SR_TXE)); // Wait while transmit buffer is empty
	while (!(SPI1->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty
	uint16_t data = SPI1->DR;
	while ((SPI1->SR & SPI_SR_BSY));
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

	return (uint8_t) (data >> 8);
}

void writeReg(uint8_t addr, uint8_t value){
	uint8_t reg = addr | 0x80;
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	SPI1->DR = reg | value << 8; // Send byte to SPI (TXE cleared)
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame

}

void loraStandbyMode(void){
	uint8_t mode = readReg(RH_RF95_REG_01_OP_MODE) & 0x3;
	if((mode != RH_RF95_MODE_STDBY)){
		writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	}
	else{
		writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	}
}

int LoRa_Init(void){
	//reset LoRa
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	//initialization
	writeReg(RH_RF95_REG_00_FIFO, 0x00);
	writeReg(RH_RF95_REG_01_OP_MODE, 0x80); //long range mode
	writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x01);
	writeReg(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0x00); //tx base addr to 0
	writeReg(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0x00); //rx base addr to 0
	writeReg(RH_RF95_REG_1D_MODEM_CONFIG1, 0x72); //coding rate and modem config
	writeReg(RH_RF95_REG_1E_MODEM_CONFIG2, 0x70); //rxpayloadcrc and spreading factor CHANGED 74
	writeReg(RH_RF95_REG_26_MODEM_CONFIG3, 0x04); //LNA gain
	writeReg(RH_RF95_REG_20_PREAMBLE_MSB, 0x00); //preamble MSB
	writeReg(RH_RF95_REG_21_PREAMBLE_LSB, 0x08); //premamble LSB
	writeReg(RH_RF95_REG_4D_PA_DAC, 0x84); //padac CHanged 04
	writeReg(RH_RF95_REG_09_PA_CONFIG, 0x8f); //output power and PA_BOOST // CHANGED 88

	//set frequency to 915MHz
	writeReg(RH_RF95_REG_06_FRF_MSB, 0xE4); //freq msb CHANGE BACK TOE4
	writeReg(RH_RF95_REG_07_FRF_MID, 0xC0); //freq mid
	writeReg(RH_RF95_REG_08_FRF_LSB, 0x00); //freq lsb

	//set power
	writeReg(RH_RF95_REG_4D_PA_DAC, 0x07); //padac
	writeReg(RH_RF95_REG_09_PA_CONFIG, 0x8F); //output power and PA_BOOST

	//set up for RX by default
	writeReg(RH_RF95_REG_01_OP_MODE, 0x05);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 1 << 6);

	uint8_t ver = readReg(0x42);
	if(ver == 0x12){
		return 1;
	}
	return 0;
}

void loraTransmit(uint8_t *data, uint8_t length){
	writeReg(RH_RF95_REG_01_OP_MODE, 0x01); //STDBY
	uint8_t txPtrBase = readReg(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR);
	writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, txPtrBase); //fifo addr pointer

	for(int i = 0; i < length; i++){
		writeReg(REG_FIFO, data[i]);
	}
	writeReg(RH_RF95_REG_22_PAYLOAD_LENGTH, length);

	writeReg(RH_RF95_REG_01_OP_MODE, 0x03); //TX Mode
	//writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x40); //DIO0

	while(readReg(RH_RF95_REG_12_IRQ_FLAGS) != 0x08);

	writeReg(RH_RF95_REG_01_OP_MODE, 0x01); //STDBY

	writeReg(RH_RF95_REG_12_IRQ_FLAGS, 0xFF); //clear txdone
}


void readFIFO(uint8_t buf[], uint16_t size)
{
	uint8_t reg = RH_RF95_REG_00_FIFO & ~0x80;
	for(int i = 0; i < size; i++){
		buf[i] = readReg(reg);
	}
}

void loraReceive(uint8_t *buf){
	writeReg(RH_RF95_REG_01_OP_MODE, 0x01);
	writeReg(RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
	if (1 || readReg(RH_RF95_REG_12_IRQ_FLAGS) == 0x00)
	{
		writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, readReg(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR)); //fifo addr ptr = fifo rx current addr
		uint8_t bytesLimit = readReg(RH_RF95_REG_13_RX_NB_BYTES);
		//HAL_Delay(10);
		readFIFO(buf, (uint16_t) bytesLimit);
		writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00);
	}
	writeReg(RH_RF95_REG_01_OP_MODE, 0x05);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x00);
}

void loraReceiveModeInit(void){
	loraStandbyMode(); // make sure mode is standby or sleep
	writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS | RH_RF95_LONG_RANGE_MODE);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x00);
}

