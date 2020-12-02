/*
 * LoRa.c
 *
 *  Created on: Nov 21, 2020
 *      Author: carrie
 */
#include "LoRa.h"
#include "main.h"

uint8_t readReg(uint8_t addr);
void writeReg(uint8_t addr, uint8_t value);
void loraWriteFIFO(uint8_t *buffer, uint8_t len);
void loraStandbyMode(void);

void LoRa_Init(void){
	//reset LoRa
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	//initialization
	//writeReg(RH_RF95_REG_00_FIFO, 0x00);
	writeReg(RH_RF95_REG_01_OP_MODE, 0x80); //long range mode
	//readReg(RH_RF95_REG_01_OP_MODE);
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
}

uint8_t readReg(uint8_t addr){
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	SPI1->DR = addr; // Send byte to SPI (TXE cleared)
	while (!(SPI1->SR & SPI_SR_TXE)); // Wait while receive buffer is empty
	while (!(SPI1->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

	return SPI1->DR & 0xFF; // Return received byte
}

char readCharReg(uint8_t addr){
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	while (!(SPI1->SR & SPI_SR_TXE)); // Wait while receive buffer is empty
	SPI1->DR = addr; // Send byte to SPI (TXE cleared)
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

	return (SPI1->DR & 0xFF); // Return received byte
}

void writeReg(uint8_t addr, uint8_t value){
	uint8_t reg = addr | 0x80;
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	SPI1->DR = reg | value << 8; // Send byte to SPI (TXE cleared)
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame
}

void loraTransmitCopy(uint8_t *buffer, uint8_t len){
	// enter standby mode
	loraStandbyMode();
	writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00); //fifo addr pointer

	for(int i = 0; i < len; i++){
		writeReg(REG_FIFO, buffer[i]);
	}
	writeReg(RH_RF95_REG_22_PAYLOAD_LENGTH, len);
	writeReg(RH_RF95_REG_01_OP_MODE, 0x03);

	//while(readReg(RH_RF95_REG_12_IRQ_FLAGS) & 0x08);
	HAL_Delay(50);
	writeReg(RH_RF95_REG_01_OP_MODE, 0x01); //STDBY

	writeReg(RH_RF95_REG_12_IRQ_FLAGS, 0xFF); //clear txdone

	//set up for RX by default
	writeReg(RH_RF95_REG_01_OP_MODE, 0x05);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x00);
}

void loraTransmit(uint8_t *buffer, uint8_t len){
	// enter standby mode
	loraStandbyMode();

	// Start SPI transaction, send address
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	loraWriteFIFO(buffer, len);
	writeReg(RH_RF95_REG_22_PAYLOAD_LENGTH, len);

	// End SPI transaction
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

	// enter transmit mode
	writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX | RH_RF95_LONG_RANGE_MODE);

	// watch txdone irq
	uint8_t data = readReg(RH_RF95_REG_12_IRQ_FLAGS);
	while(data == 0x08){
		data = readReg(RH_RF95_REG_12_IRQ_FLAGS);
	}
	// should be standby mode again
	while(readReg(RH_RF95_REG_01_OP_MODE) == 0x83);
	loraStandbyMode();

	return;
}


void loraWriteFIFO(uint8_t *buffer, uint8_t len){
	// set fifoptraddr to fifotxptrbase
	uint8_t txPtrBase = readReg(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR);
	readReg(RH_RF95_REG_0D_FIFO_ADDR_PTR);
	writeReg(RH_RF95_REG_22_PAYLOAD_LENGTH, 0);
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
	uint8_t val0;
	uint8_t val1;
	while(len > 0){
		val0 = *((uint8_t *)buffer);
		buffer += sizeof(uint8_t);
		len--;
		val1 = 0;
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

void loraStandbyMode(void){
	uint8_t mode = readReg(RH_RF95_REG_01_OP_MODE) & 0x3;
	if((mode != RH_RF95_MODE_STDBY)){
		writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	}
}
void loraReadFIFO(uint8_t *buf, uint16_t len);

void readFIFO(uint8_t buf[], uint16_t size)
{
	uint8_t reg = RH_RF95_REG_00_FIFO & ~0x80;
	while (!(SPI1->SR & SPI_SR_TXE)); // Wait while receive buffer is empty
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
	readReg(reg);
	int i = 0;
	while(i < size){
		buf[i] = SPI1->DR;
		i++;
	}
	while ((SPI1->SR & SPI_SR_BSY)); // Wait while receive buffer is empty
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame
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



void loraReceiveGPSData(uint8_t *buf){
	writeReg(RH_RF95_REG_12_IRQ_FLAGS, RH_RF95_RX_DONE_MASK);

	int currAddr = readReg(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR);
	writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, currAddr);
	uint8_t len = readReg(RH_RF95_REG_13_RX_NB_BYTES);

	/*int i = 0;
	readReg(0x00);
	while(i < len && (SPI1->SR & SPI_SR_RXNE)){
		buf[i] = SPI1->DR;
		i++;
	}*/
	loraReadFIFO(buf, (uint16_t) len);
	writeReg(RH_RF95_REG_01_OP_MODE, 0x01);

	writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00);
	//loraReceiveModeInit();
}

void loraReceiveModeInit(void){
	loraStandbyMode(); // make sure mode is standby or sleep
	writeReg(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS | RH_RF95_LONG_RANGE_MODE);
	writeReg(RH_RF95_REG_40_DIO_MAPPING1, 0x00);
}

void loraReadFIFO(uint8_t *buf, uint16_t len){

  // Start SPI transaction, send address
	uint8_t reg = RH_RF95_REG_00_FIFO & ~0x80;
	readReg(reg);
	while ((SPI1->SR & SPI_SR_BSY));
	for(int i = 0; i < len; i++){
		buf[i] = readReg(REG_FIFO);
	}
	//HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS high to end frame
	//for(int i = 0; i < len; i++){
		//buf[i] = readReg(REG_FIFO);
	//}
  // End SPI transaction
	//HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame
	return;
}


