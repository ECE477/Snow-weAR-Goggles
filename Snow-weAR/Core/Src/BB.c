#include "BB.h"
#include "BQ27441_Defs.h"
#include "main.h"

void BB_Init() {
    _deviceAddress = BQ72441_I2C_ADDRESS;
    _sealFlag = false;
    _userConfigControl = false;
}

bool BB_begin(I2C_HandleTypeDef *hi2c) {
	uint16_t deviceID = 0;


	deviceID = BB_deviceType(hi2c); // Read deviceType from BQ27441

	//return deviceID;
	if (deviceID == BQ27441_DEVICE_ID) {
		return true; // If device ID is valid, return true
	}

	return false; // Otherwise return false*/
}

uint16_t readVoltage(I2C_HandleTypeDef *hi2c) {
	return readWord(BQ27441_COMMAND_VOLTAGE, hi2c);
}

uint16_t readCapacity(I2C_HandleTypeDef *hi2c, capacity_measure type) {
	uint16_t capacity = 0;
	switch (type) {
		case 0:
			return readWord(BQ27441_COMMAND_REM_CAPACITY, hi2c);
			break;
		case 1:
			return readWord(BQ27441_COMMAND_FULL_CAPACITY, hi2c);
			break;
		case 2:
			capacity = readWord(BQ27441_COMMAND_NOM_CAPACITY, hi2c);
			break;
		case 3:
			capacity = readWord(BQ27441_COMMAND_AVAIL_CAPACITY, hi2c);
			break;
		case 4:
			capacity = readWord(BQ27441_COMMAND_REM_CAP_FIL, hi2c);
			break;
		case 5:
			capacity = readWord(BQ27441_COMMAND_REM_CAP_UNFL, hi2c);
			break;
		case 6:
			capacity = readWord(BQ27441_COMMAND_FULL_CAP_FIL, hi2c);
			break;
		case 7:
			capacity = readWord(BQ27441_COMMAND_FULL_CAP_UNFL, hi2c);
			break;
		case 8:
			capacity = readWord(BQ27441_EXTENDED_CAPACITY, hi2c);
	}
	return capacity;
}

int16_t readAvgPower(I2C_HandleTypeDef *hi2c) {
	return (int16_t) readWord(BQ27441_COMMAND_AVG_POWER, hi2c);
}

uint16_t readTemp(I2C_HandleTypeDef *hi2c) {
	return 10*((readWord(BQ27441_COMMAND_TEMP, hi2c)/10 - 273.15) * 9/5 + 32);
}

uint16_t BB_deviceType(I2C_HandleTypeDef *hi2c) {
	return readControlWord(BQ27441_CONTROL_DEVICE_TYPE, hi2c);
}

uint16_t readControlWord(uint16_t function, I2C_HandleTypeDef *hi2c) {
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	uint8_t data[2] = {0, 0};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hi2c, BB_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, command, 2, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		ret = HAL_I2C_Mem_Read(hi2c, BB_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &data, 2, HAL_MAX_DELAY);
		if(ret == HAL_OK)
			return ((uint16_t)data[1] << 8) | data[0];
	}
	return 100;
}

uint16_t readWord(uint16_t subAddress, I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef ret;
	uint8_t data[2];
	ret = HAL_I2C_Mem_Read(hi2c, BB_I2C_ADDR, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 2, HAL_MAX_DELAY);
	if(ret == HAL_OK)
		return ((uint16_t)data[1] << 8) | data[0];
	return 101;
}

static bool executeControlWord(uint16_t function, I2C_HandleTypeDef *hi2c) {
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hi2c, BB_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, command, 2, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		return true;
	}
	return false;
}

bool setCap(I2C_HandleTypeDef *hi2c, uint16_t capacity) {
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x0A (10)
	// Design capacity is a 2-byte piece of data - MSB first
	uint8_t capMSB = capacity >> 8;
	uint8_t capLSB = capacity & 0x00FF;
	uint8_t capacityData[2] = {capMSB, capLSB};
	return writeExtendedData(hi2c, BQ27441_ID_STATE, 10, capacityData, 2);
}

static bool writeExtendedData(I2C_HandleTypeDef *hi2c, uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len) {
	if (len > 32)
		return false;
	if (!_userConfigControl)
		BB_enterConfig(hi2c, false);
	if (!blockDataControl(hi2c)) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(hi2c, classID)) // Write class ID using DataBlockClass()
		return false;

	blockDataOffset(hi2c, offset / 32); // Write 32-bit block offset (usually 0)
	computeBlockChecksum(hi2c); // Compute checksum going in
	//uint8_t oldCsum = blockDataChecksum(dev);

	// Write data bytes:
	for (int i = 0; i < len; i++) {
		// Write to offset, mod 32 if offset is greater than 32
		// The blockDataOffset above sets the 32-bit block
		writeBlockData(hi2c, (offset % 32) + i, data[i]);
	}

	// Write new checksum using BlockDataChecksum (0x60)
	uint8_t newCsum = computeBlockChecksum(hi2c); // Compute the new checksum
	writeBlockChecksum(hi2c, newCsum);
	if (!_userConfigControl) BB_exitConfig(hi2c, true);
	return true;
}

static uint8_t readExtendedData(I2C_HandleTypeDef *hi2c, uint8_t classID, uint8_t offset) {
	uint8_t retData = 0;
	if (!_userConfigControl)
		BB_enterConfig(hi2c, false);
	if (!blockDataControl(hi2c)) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(hi2c, classID)) // Write class ID using DataBlockClass()
		return false;

	blockDataOffset(hi2c, offset / 32); // Write 32-bit block offset (usually 0)

	computeBlockChecksum(hi2c); // Compute checksum going in
	//uint8_t oldCsum = blockDataChecksum(dev);
	/*for (int i=0; i<32; i++)
		Serial.print(String(readBlockData(i)) + " ");*/
	retData = readBlockData(hi2c, offset % 32); // Read from offset (limit to 0-31)

	if (!_userConfigControl)
		BB_exitConfig(hi2c, true);
	return retData;
}

bool BB_enterConfig(I2C_HandleTypeDef *hi2c, bool userControl) {
	if (userControl) _userConfigControl = true;
	if (sealed(hi2c)) {
		_sealFlag = true;
		unseal(hi2c); // Must be unsealed before making changes
	}

	if (executeControlWord(BQ27441_CONTROL_SET_CFGUPDATE, hi2c)) {
		int16_t timeout = BQ72441_I2C_TIMEOUT;
		while ((timeout--) && (!(BB_flags(hi2c) & BQ27441_FLAG_CFGUPMODE)))
			HAL_Delay(1);

		if (timeout > 0)
			return true;
	}

	return false;
}

static bool sealed(I2C_HandleTypeDef *hi2c) {
	uint16_t stat = BB_status(hi2c);
	return stat & BQ27441_STATUS_SS;
}

static bool seal(I2C_HandleTypeDef *hi2c) {
	return readControlWord(BQ27441_CONTROL_SEALED, hi2c);
}

// UNseal the BQ27441-G1A
static bool unseal(I2C_HandleTypeDef *hi2c) {
	// To unseal the BQ27441, write the key to the control
	// command. Then immediately write the same key to control again.
	if (readControlWord(BQ27441_UNSEAL_KEY, hi2c)) {
		return readControlWord(BQ27441_UNSEAL_KEY, hi2c);
	}
	return false;
}

uint16_t BB_status(I2C_HandleTypeDef *hi2c) {
	return readControlWord(BQ27441_CONTROL_STATUS, hi2c);
}

uint16_t BB_flags(I2C_HandleTypeDef *hi2c) {
	return readWord(BQ27441_COMMAND_FLAGS, hi2c);
}

bool BB_exitConfig(I2C_HandleTypeDef *hi2c, bool resim) {
	// There are two methods for exiting config mode:
	//    1. Execute the EXIT_CFGUPDATE command
	//    2. Execute the SOFT_RESET command
	// EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
	// measurement, and without resimulating to update unfiltered-SoC and SoC.
	// If a new OCV measurement or resimulation is desired, SOFT_RESET or
	// EXIT_RESIM should be used to exit config mode.
	if (resim) {
		if (softReset(hi2c)) {
			int16_t timeout = BQ72441_I2C_TIMEOUT;
			while ((timeout--) && ((BB_flags(hi2c) & BQ27441_FLAG_CFGUPMODE)))
				HAL_Delay(1);
			if (timeout > 0) {
				if (_sealFlag) seal(hi2c); // Seal back up if we IC was sealed coming in
				return true;
			}
		}
		return false;
	}
	else {
		return executeControlWord(BQ27441_CONTROL_EXIT_CFGUPDATE, hi2c);
	}
}

static bool softReset(I2C_HandleTypeDef *hi2c) {
	return executeControlWord(BQ27441_CONTROL_SOFT_RESET, hi2c);
}

static bool blockDataControl(I2C_HandleTypeDef *hi2c) {
	uint8_t enableByte = 0x00;
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hi2c, BB_I2C_ADDR, BQ27441_EXTENDED_CONTROL, I2C_MEMADD_SIZE_8BIT, &enableByte, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		return true;
	}
	return false;
}

static bool blockDataClass(I2C_HandleTypeDef *hi2c, uint8_t id) {
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hi2c, BB_I2C_ADDR, BQ27441_EXTENDED_DATACLASS, I2C_MEMADD_SIZE_8BIT, &id, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		return true;
	}
	return false;
}

static bool blockDataOffset(I2C_HandleTypeDef *hi2c, uint8_t offset) {
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hi2c, BB_I2C_ADDR, BQ27441_EXTENDED_DATABLOCK, I2C_MEMADD_SIZE_8BIT, &offset, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		return true;
	}
	return false;
}

// Read the current checksum using BlockDataCheckSum()
static uint8_t blockDataChecksum(I2C_HandleTypeDef *hi2c) {
	uint8_t csum;
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(hi2c, BB_I2C_ADDR, BQ27441_EXTENDED_CHECKSUM, I2C_MEMADD_SIZE_8BIT, &csum, 1, HAL_MAX_DELAY);
	return csum;
}

static uint8_t computeBlockChecksum(I2C_HandleTypeDef *hi2c) {
	uint8_t data[32];
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(hi2c, BB_I2C_ADDR, BQ27441_EXTENDED_BLOCKDATA, I2C_MEMADD_SIZE_8BIT, data, 32, HAL_MAX_DELAY);

	uint8_t csum = 0;
	for (int i=0; i<32; i++) {
		csum += data[i];
	}
	csum = 255 - csum;

	return csum;
}

static bool writeBlockChecksum(I2C_HandleTypeDef *hi2c, uint8_t csum) {
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hi2c, BB_I2C_ADDR, BQ27441_EXTENDED_CHECKSUM, I2C_MEMADD_SIZE_8BIT, &csum, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		return true;
	}
	return false;
}

static bool writeBlockData(I2C_HandleTypeDef *hi2c, uint8_t offset, uint8_t data) {
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hi2c, BB_I2C_ADDR, address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK) {
		return true;
	}
	return false;
}

uint16_t BB_soc(I2C_HandleTypeDef *hi2c, soc_measure type) {
	uint16_t socRet = 0;
	switch (type) {
	case 0:
		socRet = readWord(BQ27441_COMMAND_SOC, hi2c);
		break;
	case 1:
		socRet = readWord(BQ27441_COMMAND_SOC_UNFL, hi2c);
		break;
	}

	return socRet;
}

int16_t BB_current(I2C_HandleTypeDef *hi2c, current_measure type) {
	int16_t current = 0;
	switch (type) {
	case 0:
		current = (int16_t) readWord(BQ27441_COMMAND_AVG_CURRENT, hi2c);
		break;
	case 1:
		current = (int16_t) readWord(BQ27441_COMMAND_STDBY_CURRENT, hi2c);
		break;
	case 2:
		current = (int16_t) readWord(BQ27441_COMMAND_MAX_CURRENT, hi2c);
		break;
	}

	return current;
}

uint8_t BB_soh(I2C_HandleTypeDef *hi2c, soh_measure type) {
	uint16_t sohRaw = readWord(BQ27441_COMMAND_SOH, hi2c);
	uint8_t sohStatus = sohRaw >> 8;
	uint8_t sohPercent = sohRaw & 0x00FF;

	if (type == 0)
		return sohPercent;
	else
		return sohStatus;
}

bool BB_GPOUTPolarity(I2C_HandleTypeDef *hi2c) {
	uint16_t opConfigRegister = opConfig(hi2c);

	return (opConfigRegister & BQ27441_OPCONFIG_GPIOPOL);
}

// Set GPOUT polarity to active-high or active-low
bool BB_setGPOUTPolarity(I2C_HandleTypeDef *hi2c, bool activeHigh) {
	uint16_t oldOpConfig = opConfig(hi2c);

	// Check to see if we need to update opConfig:
	if ((activeHigh && (oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)) ||
        (!activeHigh && !(oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)))
		return true;

	uint16_t newOpConfig = oldOpConfig;
	if (activeHigh)
		newOpConfig |= BQ27441_OPCONFIG_GPIOPOL;
	else
		newOpConfig &= ~(BQ27441_OPCONFIG_GPIOPOL);

	return writeOpConfig(hi2c, newOpConfig);
}

// Get GPOUT function (BAT_LOW or SOC_INT)
bool BB_GPOUTFunction(I2C_HandleTypeDef *hi2c) {
	uint16_t opConfigRegister = opConfig(hi2c);
	return (opConfigRegister & BQ27441_OPCONFIG_BATLOWEN);
}

// Set GPOUT function to BAT_LOW or SOC_INT
bool BB_setGPOUTFunction(I2C_HandleTypeDef *hi2c, gpout_function function) {
	uint16_t oldOpConfig = opConfig(hi2c);

	// Check to see if we need to update opConfig:
	if ((function && (oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)) ||
        (!function && !(oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)))
		return true;

	// Modify BATLOWN_EN bit of opConfig:
	uint16_t newOpConfig = oldOpConfig;
	if (function)
		newOpConfig |= BQ27441_OPCONFIG_BATLOWEN;
	else
		newOpConfig &= ~(BQ27441_OPCONFIG_BATLOWEN);

	// Write new opConfig
	return writeOpConfig(hi2c, newOpConfig);
}

// Get SOC1_Set Threshold - threshold to set the alert flag
uint8_t BB_SOC1SetThreshold(I2C_HandleTypeDef *hi2c) {
	return readExtendedData(hi2c, BQ27441_ID_DISCHARGE, 0);
}

// Get SOC1_Clear Threshold - threshold to clear the alert flag
uint8_t BB_SOC1ClearThreshold(I2C_HandleTypeDef *hi2c) {
	return readExtendedData(hi2c, BQ27441_ID_DISCHARGE, 1);
}

// Set the SOC1 set and clear thresholds to a percentage
bool BB_setSOC1Thresholds(I2C_HandleTypeDef *hi2c, uint8_t set, uint8_t clear) {
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(hi2c, BQ27441_ID_DISCHARGE, 0, thresholds, 2);
}

// Get SOCF_Set Threshold - threshold to set the alert flag
uint8_t BB_SOCFSetThreshold(I2C_HandleTypeDef *hi2c) {
	return readExtendedData(hi2c, BQ27441_ID_DISCHARGE, 2);
}

// Get SOCF_Clear Threshold - threshold to clear the alert flag
uint8_t BB_SOCFClearThreshold(I2C_HandleTypeDef *hi2c) {
	return readExtendedData(hi2c, BQ27441_ID_DISCHARGE, 3);
}

// Set the SOCF set and clear thresholds to a percentage
bool BB_setSOCFThresholds(I2C_HandleTypeDef *hi2c, uint8_t set, uint8_t clear) {
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(hi2c, BQ27441_ID_DISCHARGE, 2, thresholds, 2);
}

// Check if the SOC1 flag is set
bool BB_socFlag(I2C_HandleTypeDef *hi2c) {
	uint16_t flagState = BB_flags(hi2c);
	return flagState & BQ27441_FLAG_SOC1;
}

// Check if the SOCF flag is set
bool BB_socfFlag(I2C_HandleTypeDef *hi2c) {
	uint16_t flagState = BB_flags(hi2c);
	return flagState & BQ27441_FLAG_SOCF;
}

// Get the SOC_INT interval delta
uint8_t BB_sociDelta(I2C_HandleTypeDef *hi2c) {
	return readExtendedData(hi2c, BQ27441_ID_STATE, 26);
}

// Set the SOC_INT interval delta to a value between 1 and 100
bool BB_setSOCIDelta(I2C_HandleTypeDef *hi2c, uint8_t delta) {
	uint8_t soci = constrain(delta, 0, 100);
	return writeExtendedData(hi2c, BQ27441_ID_STATE, 26, &soci, 1);
}

// Pulse the GPOUT pin - must be in SOC_INT mode
bool BB_pulseGPOUT(I2C_HandleTypeDef *hi2c) {
	return executeControlWord(BQ27441_CONTROL_PULSE_SOC_INT, hi2c);
}

static int32_t constrain(int32_t x, int32_t a, int32_t b) {
    if(x < a)
        x = a;
    if(a > b)
        x = b;
    return x;
}
