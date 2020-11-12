/******************************************************************************
SparkFunBQ27441.cpp
BQ27441 Arduino Library Main Source File
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Implementation of all features of the BQ27441 LiPo Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/

#include "BQ27441.h"
#include "BQ27441_Defs.h"

static uint8_t _deviceAddress;  // Stores the BQ27441-G1A's I2C address
static bool _sealFlag; // Global to identify that IC was previously sealed
static bool _userConfigControl; // Global to identify that user has control over 
                            // entering/exiting config

/**
    Check if the BQ27441-G1A is sealed or not.
    
    @return true if the chip is sealed
*/
static bool sealed(bq27441_dev_t *dev);

/**
    Seal the BQ27441-G1A
    
    @return true on success
*/
static bool seal(bq27441_dev_t *dev);

/**
    UNseal the BQ27441-G1A
    
    @return true on success
*/
static bool unseal(bq27441_dev_t *dev);
    
/**
    Read the 16-bit opConfig register from extended data
    
    @return opConfig register contents
*/
static uint16_t opConfig(bq27441_dev_t *dev);

/**
    Write the 16-bit opConfig register in extended data
    
    @param New 16-bit value for opConfig
    @return true on success
*/	
static bool writeOpConfig(bq27441_dev_t *dev, uint16_t value);

/**
    Issue a soft-reset to the BQ27441-G1A
    
    @return true on success
*/	
static bool softReset(bq27441_dev_t *dev);

/**
    Read a 16-bit command word from the BQ27441-G1A
    
    @param subAddress is the command to be read from
    @return 16-bit value of the command's contents
*/	
static uint16_t readWord(bq27441_dev_t *dev, uint16_t subAddress);

/**
    Read a 16-bit subcommand() from the BQ27441-G1A's control()
    
    @param function is the subcommand of control() to be read
    @return 16-bit value of the subcommand's contents
*/	
static uint16_t readControlWord(bq27441_dev_t *dev, uint16_t function);

/**
    Execute a subcommand() from the BQ27441-G1A's control()
    
    @param function is the subcommand of control() to be executed
    @return true on success
*/	
static bool executeControlWord(bq27441_dev_t *dev, uint16_t function);

////////////////////////////
// Extended Data Commands //
////////////////////////////
/**
    Issue a BlockDataControl() command to enable BlockData access
    
    @return true on success
*/
static bool blockDataControl(bq27441_dev_t *dev);

/**
    Issue a DataClass() command to set the data class to be accessed
    
    @param id is the id number of the class
    @return true on success
*/
static bool blockDataClass(bq27441_dev_t *dev, uint8_t id);

/**
    Issue a DataBlock() command to set the data block to be accessed
    
    @param offset of the data block
    @return true on success
*/
static bool blockDataOffset(bq27441_dev_t *dev, uint8_t offset);

/**
    Read the current checksum using BlockDataCheckSum()
    
    @return true on success
*/
static uint8_t blockDataChecksum(bq27441_dev_t *dev);

/**
    Use BlockData() to read a byte from the loaded extended data
    
    @param offset of data block byte to be read
    @return true on success
*/
static uint8_t readBlockData(bq27441_dev_t *dev, uint8_t offset);

/**
    Use BlockData() to write a byte to an offset of the loaded data
    
    @param offset is the position of the byte to be written
            data is the value to be written
    @return true on success
*/
static bool writeBlockData(bq27441_dev_t *dev, uint8_t offset, uint8_t data);

/**
    Read all 32 bytes of the loaded extended data and compute a 
    checksum based on the values.
    
    @return 8-bit checksum value calculated based on loaded data
*/
static uint8_t computeBlockChecksum(bq27441_dev_t *dev);

/**
    Use the BlockDataCheckSum() command to write a checksum value
    
    @param csum is the 8-bit checksum to be written
    @return true on success
*/
static bool writeBlockChecksum(bq27441_dev_t *dev, uint8_t csum);

/**
    Read a byte from extended data specifying a class ID and position offset
    
    @param classID is the id of the class to be read from
            offset is the byte position of the byte to be read
    @return 8-bit value of specified data
*/
static uint8_t readExtendedData(bq27441_dev_t *dev, uint8_t classID, uint8_t offset);

/**
    Write a specified number of bytes to extended data specifying a 
    class ID, position offset.
    
    @param classID is the id of the class to be read from
            offset is the byte position of the byte to be read
            data is the data buffer to be written
            len is the number of bytes to be written
    @return true on success
*/
static bool writeExtendedData(bq27441_dev_t *dev, uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len);


static int32_t constrain(int32_t x, int32_t a, int32_t b);
/////////////////////////////////
// I2C Read and Write Routines //
/////////////////////////////////

// /**
//     Read a specified number of bytes over I2C at a given subAddress
    
//     @param subAddress is the 8-bit address of the data to be read
//             dest is the data buffer to be written to
//             count is the number of bytes to be read
//     @return true on success
// */
// static int16_t i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

// /**
//     Write a specified number of bytes over I2C to a given subAddress
    
//     @param subAddress is the 8-bit address of the data to be written to
//             src is the data buffer to be written
//             count is the number of bytes to be written
//     @return true on success
// */
// static uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count);





/*****************************************************************************
 ************************** Initialization Functions *************************
 *****************************************************************************/
void BQ27441_Init(bq27441_dev_t *dev)
{
    _deviceAddress = BQ72441_I2C_ADDRESS;
    _sealFlag = false;
    _userConfigControl = false;
}



// Initializes I2C and verifies communication with the BQ27441.
bool BQ27441_begin(bq27441_dev_t *dev)
{
	uint16_t deviceID = 0;
	

	deviceID = BQ27441_deviceType(dev); // Read deviceType from BQ27441
	
	if (deviceID == BQ27441_DEVICE_ID)
	{
		return true; // If device ID is valid, return true
	}
	
	return false; // Otherwise return false
}

// Configures the design capacity of the connected battery.
bool BQ27441_setCapacity(bq27441_dev_t *dev, uint16_t capacity)
{
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x0A (10)
	// Design capacity is a 2-byte piece of data - MSB first
	uint8_t capMSB = capacity >> 8;
	uint8_t capLSB = capacity & 0x00FF;
	uint8_t capacityData[2] = {capMSB, capLSB};
	return writeExtendedData(dev, BQ27441_ID_STATE, 10, capacityData, 2);
}

/*****************************************************************************
 ********************** Battery Characteristics Functions ********************
 *****************************************************************************/

// Reads and returns the battery voltage
uint16_t BQ27441_voltage(bq27441_dev_t *dev)
{
	return readWord(dev, BQ27441_COMMAND_VOLTAGE);
}

// Reads and returns the specified current measurement
int16_t BQ27441_current(bq27441_dev_t *dev, current_measure type)
{
	int16_t current = 0;
	switch (type)
	{
	case AVG:
		current = (int16_t) readWord(dev,BQ27441_COMMAND_AVG_CURRENT);
		break;
	case STBY:
		current = (int16_t) readWord(dev,BQ27441_COMMAND_STDBY_CURRENT);
		break;
	case MAX:
		current = (int16_t) readWord(dev, BQ27441_COMMAND_MAX_CURRENT);
		break;
	}
	
	return current;
}

// Reads and returns the specified capacity measurement
uint16_t BQ27441_capacity(bq27441_dev_t *dev, capacity_measure type)
{
	uint16_t capacity = 0;
	switch (type)
	{
	case REMAIN:
		return readWord(dev, BQ27441_COMMAND_REM_CAPACITY);
		break;
	case FULL:
		return readWord(dev, BQ27441_COMMAND_FULL_CAPACITY);
		break;
	case AVAIL:
		capacity = readWord(dev, BQ27441_COMMAND_NOM_CAPACITY);
		break;
	case AVAIL_FULL:
		capacity = readWord(dev, BQ27441_COMMAND_AVAIL_CAPACITY);
		break;
	case REMAIN_F: 
		capacity = readWord(dev, BQ27441_COMMAND_REM_CAP_FIL);
		break;
	case REMAIN_UF:
		capacity = readWord(dev, BQ27441_COMMAND_REM_CAP_UNFL);
		break;
	case FULL_F:
		capacity = readWord(dev, BQ27441_COMMAND_FULL_CAP_FIL);
		break;
	case FULL_UF:
		capacity = readWord(dev, BQ27441_COMMAND_FULL_CAP_UNFL);
		break;
	case DESIGN:
		capacity = readWord(dev, BQ27441_EXTENDED_CAPACITY);
	}
	
	return capacity;
}

// Reads and returns measured average power
int16_t BQ27441_power(bq27441_dev_t *dev)
{
	return (int16_t) readWord(dev, BQ27441_COMMAND_AVG_POWER);
}

// Reads and returns specified state of charge measurement
uint16_t BQ27441_soc(bq27441_dev_t *dev, soc_measure type)
{
	uint16_t socRet = 0;
	switch (type)
	{
	case FILTERED:
		socRet = readWord(dev, BQ27441_COMMAND_SOC);
		break;
	case UNFILTERED:
		socRet = readWord(dev, BQ27441_COMMAND_SOC_UNFL);
		break;
	}
	
	return socRet;
}

// Reads and returns specified state of health measurement
uint8_t BQ27441_soh(bq27441_dev_t *dev, soh_measure type)
{
	uint16_t sohRaw = readWord(dev, BQ27441_COMMAND_SOH);
	uint8_t sohStatus = sohRaw >> 8;
	uint8_t sohPercent = sohRaw & 0x00FF;
	
	if (type == PERCENT)	
		return sohPercent;
	else
		return sohStatus;
}

// Reads and returns specified temperature measurement
uint16_t BQ27441_temperature(bq27441_dev_t *dev, temp_measure type)
{
	uint16_t temp = 0;
	switch (type)
	{
	case BATTERY:
		temp = readWord(dev, BQ27441_COMMAND_TEMP);
		break;
	case INTERNAL_TEMP:
		temp = readWord(dev, BQ27441_COMMAND_INT_TEMP);
		break;
	}
	return temp;
}

/*****************************************************************************
 ************************** GPOUT Control Functions **************************
 *****************************************************************************/
// Get GPOUT polarity setting (active-high or active-low)
bool BQ27441_GPOUTPolarity(bq27441_dev_t *dev)
{
	uint16_t opConfigRegister = opConfig(dev);
	
	return (opConfigRegister & BQ27441_OPCONFIG_GPIOPOL);
}

// Set GPOUT polarity to active-high or active-low
bool BQ27441_setGPOUTPolarity(bq27441_dev_t *dev, bool activeHigh)
{
	uint16_t oldOpConfig = opConfig(dev);
	
	// Check to see if we need to update opConfig:
	if ((activeHigh && (oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)) ||
        (!activeHigh && !(oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)))
		return true;
		
	uint16_t newOpConfig = oldOpConfig;
	if (activeHigh)
		newOpConfig |= BQ27441_OPCONFIG_GPIOPOL;
	else
		newOpConfig &= ~(BQ27441_OPCONFIG_GPIOPOL);
	
	return writeOpConfig(dev, newOpConfig);	
}

// Get GPOUT function (BAT_LOW or SOC_INT)
bool BQ27441_GPOUTFunction(bq27441_dev_t *dev)
{
	uint16_t opConfigRegister = opConfig(dev);
	
	return (opConfigRegister & BQ27441_OPCONFIG_BATLOWEN);	
}

// Set GPOUT function to BAT_LOW or SOC_INT
bool BQ27441_setGPOUTFunction(bq27441_dev_t *dev, gpout_function function)
{
	uint16_t oldOpConfig = opConfig(dev);
	
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
	return writeOpConfig(dev, newOpConfig);	
}

// Get SOC1_Set Threshold - threshold to set the alert flag
uint8_t BQ27441_SOC1SetThreshold(bq27441_dev_t *dev)
{
	return readExtendedData(dev, BQ27441_ID_DISCHARGE, 0);
}

// Get SOC1_Clear Threshold - threshold to clear the alert flag
uint8_t BQ27441_SOC1ClearThreshold(bq27441_dev_t *dev)
{
	return readExtendedData(dev, BQ27441_ID_DISCHARGE, 1);	
}

// Set the SOC1 set and clear thresholds to a percentage
bool BQ27441_setSOC1Thresholds(bq27441_dev_t *dev, uint8_t set, uint8_t clear)
{
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(dev, BQ27441_ID_DISCHARGE, 0, thresholds, 2);
}

// Get SOCF_Set Threshold - threshold to set the alert flag
uint8_t BQ27441_SOCFSetThreshold(bq27441_dev_t *dev)
{
	return readExtendedData(dev, BQ27441_ID_DISCHARGE, 2);
}

// Get SOCF_Clear Threshold - threshold to clear the alert flag
uint8_t BQ27441_SOCFClearThreshold(bq27441_dev_t *dev)
{
	return readExtendedData(dev, BQ27441_ID_DISCHARGE, 3);	
}

// Set the SOCF set and clear thresholds to a percentage
bool BQ27441_setSOCFThresholds(bq27441_dev_t *dev, uint8_t set, uint8_t clear)
{
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(dev, BQ27441_ID_DISCHARGE, 2, thresholds, 2);
}

// Check if the SOC1 flag is set
bool BQ27441_socFlag(bq27441_dev_t *dev)
{
	uint16_t flagState = BQ27441_flags(dev);
	
	return flagState & BQ27441_FLAG_SOC1;
}

// Check if the SOCF flag is set
bool BQ27441_socfFlag(bq27441_dev_t *dev)
{
	uint16_t flagState = BQ27441_flags(dev);
	
	return flagState & BQ27441_FLAG_SOCF;
	
}

// Get the SOC_INT interval delta
uint8_t BQ27441_sociDelta(bq27441_dev_t *dev)
{
	return readExtendedData(dev, BQ27441_ID_STATE, 26);
}

// Set the SOC_INT interval delta to a value between 1 and 100
bool BQ27441_setSOCIDelta(bq27441_dev_t *dev, uint8_t delta)
{
	uint8_t soci = constrain(delta, 0, 100);
	return writeExtendedData(dev, BQ27441_ID_STATE, 26, &soci, 1);
}

// Pulse the GPOUT pin - must be in SOC_INT mode
bool BQ27441_pulseGPOUT(bq27441_dev_t *dev)
{
	return executeControlWord(dev, BQ27441_CONTROL_PULSE_SOC_INT);
}

/*****************************************************************************
 *************************** Control Sub-Commands ****************************
 *****************************************************************************/

// Read the device type - should be 0x0421
uint16_t BQ27441_deviceType(bq27441_dev_t *dev)
{
	return readControlWord(dev, BQ27441_CONTROL_DEVICE_TYPE);
}

// Enter configuration mode - set userControl if calling from an Arduino sketch
// and you want control over when to exitConfig
bool BQ27441_enterConfig(bq27441_dev_t *dev, bool userControl)
{
	if (userControl) _userConfigControl = true;
	
	if (sealed(dev))
	{
		_sealFlag = true;
		unseal(dev); // Must be unsealed before making changes
	}
	
	if (executeControlWord(dev, BQ27441_CONTROL_SET_CFGUPDATE))
	{
		int16_t timeout = BQ72441_I2C_TIMEOUT;
		while ((timeout--) && (!(BQ27441_flags(dev) & BQ27441_FLAG_CFGUPMODE)))
			dev->delay_ms(1);
		
		if (timeout > 0)
			return true;
	}
	
	return false;
}

// Exit configuration mode with the option to perform a resimulation
bool BQ27441_exitConfig(bq27441_dev_t *dev, bool resim)
{
	// There are two methods for exiting config mode:
	//    1. Execute the EXIT_CFGUPDATE command
	//    2. Execute the SOFT_RESET command
	// EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
	// measurement, and without resimulating to update unfiltered-SoC and SoC.
	// If a new OCV measurement or resimulation is desired, SOFT_RESET or
	// EXIT_RESIM should be used to exit config mode.
	if (resim)
	{
		if (softReset(dev))
		{
			int16_t timeout = BQ72441_I2C_TIMEOUT;
			while ((timeout--) && ((BQ27441_flags(dev) & BQ27441_FLAG_CFGUPMODE)))
				dev->delay_ms(1);
			if (timeout > 0)
			{
				if (_sealFlag) seal(dev); // Seal back up if we IC was sealed coming in
				return true;
			}
		}
		return false;
	}
	else
	{
		return executeControlWord(dev, BQ27441_CONTROL_EXIT_CFGUPDATE);
	}	
}

// Read the flags() command
uint16_t BQ27441_flags(bq27441_dev_t *dev)
{
	return readWord(dev, BQ27441_COMMAND_FLAGS);
}

// Read the CONTROL_STATUS subcommand of control()
uint16_t BQ27441_status(bq27441_dev_t *dev)
{
	return readControlWord(dev, BQ27441_CONTROL_STATUS);
}

/*****************************************************************************
 **************************** Private Functions ******************************
 *****************************************************************************/

// Check if the BQ27441-G1A is sealed or not.
static bool sealed(bq27441_dev_t *dev)
{
	uint16_t stat = BQ27441_status(dev);
	return stat & BQ27441_STATUS_SS;
}

// Seal the BQ27441-G1A
static bool seal(bq27441_dev_t *dev)
{
	return readControlWord(dev, BQ27441_CONTROL_SEALED);
}

// UNseal the BQ27441-G1A
static bool unseal(bq27441_dev_t *dev)
{
	// To unseal the BQ27441, write the key to the control
	// command. Then immediately write the same key to control again.
	if (readControlWord(dev, BQ27441_UNSEAL_KEY))
	{
		return readControlWord(dev, BQ27441_UNSEAL_KEY);
	}
	return false;
}

// Read the 16-bit opConfig register from extended data
static uint16_t opConfig(bq27441_dev_t *dev)
{
	return readWord(dev, BQ27441_EXTENDED_OPCONFIG);
}

// Write the 16-bit opConfig register in extended data
static bool writeOpConfig(bq27441_dev_t *dev, uint16_t value)
{
	uint8_t opConfigMSB = value >> 8;
	uint8_t opConfigLSB = value & 0x00FF;
	uint8_t opConfigData[2] = {opConfigMSB, opConfigLSB};
	
	// OpConfig register location: BQ27441_ID_REGISTERS id, offset 0
	return writeExtendedData(dev, BQ27441_ID_REGISTERS, 0, opConfigData, 2);	
}

// Issue a soft-reset to the BQ27441-G1A
static bool softReset(bq27441_dev_t *dev)
{
	return executeControlWord(dev, BQ27441_CONTROL_SOFT_RESET);
}

// Read a 16-bit command word from the BQ27441-G1A
static uint16_t readWord(bq27441_dev_t *dev, uint16_t subAddress)
{
	uint8_t data[2];
	dev->read(subAddress, data, 2);
	return ((uint16_t) data[1] << 8) | data[0];
}

// Read a 16-bit subcommand() from the BQ27441-G1A's control()
static uint16_t readControlWord(bq27441_dev_t *dev, uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	uint8_t data[2] = {0, 0};
	
	dev->write((uint8_t) 0, command, 2);
	
	if (dev->read((uint8_t) 0, data, 2))
	{
		return ((uint16_t)data[1] << 8) | data[0];
	}
	
	return false;
}

// Execute a subcommand() from the BQ27441-G1A's control()
static bool executeControlWord(bq27441_dev_t *dev, uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	//uint8_t data[2] = {0, 0};
	
	if (dev->write((uint8_t) 0, command, 2))
		return true;
	
	return false;
}

/*****************************************************************************
 ************************** Extended Data Commands ***************************
 *****************************************************************************/
 
// Issue a BlockDataControl() command to enable BlockData access
static bool blockDataControl(bq27441_dev_t *dev)
{
	uint8_t enableByte = 0x00;
	return dev->write(BQ27441_EXTENDED_CONTROL, &enableByte, 1);
}

// Issue a DataClass() command to set the data class to be accessed
static bool blockDataClass(bq27441_dev_t *dev, uint8_t id)
{
	return dev->write(BQ27441_EXTENDED_DATACLASS, &id, 1);
}

// Issue a DataBlock() command to set the data block to be accessed
static bool blockDataOffset(bq27441_dev_t *dev, uint8_t offset)
{
	return dev->write(BQ27441_EXTENDED_DATABLOCK, &offset, 1);
}

// Read the current checksum using BlockDataCheckSum()
static uint8_t blockDataChecksum(bq27441_dev_t *dev)
{
	uint8_t csum;
	dev->read(BQ27441_EXTENDED_CHECKSUM, &csum, 1);
	return csum;
}

// Use BlockData() to read a byte from the loaded extended data
static uint8_t readBlockData(bq27441_dev_t *dev, uint8_t offset)
{
	uint8_t ret;
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	dev->read(address, &ret, 1);
	return ret;
}

// Use BlockData() to write a byte to an offset of the loaded data
static bool writeBlockData(bq27441_dev_t *dev, uint8_t offset, uint8_t data)
{
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	return dev->write(address, &data, 1);
}

// Read all 32 bytes of the loaded extended data and compute a 
// checksum based on the values.
static uint8_t computeBlockChecksum(bq27441_dev_t *dev)
{
	uint8_t data[32];
	dev->read(BQ27441_EXTENDED_BLOCKDATA, data, 32);

	uint8_t csum = 0;
	for (int i=0; i<32; i++)
	{
		csum += data[i];
	}
	csum = 255 - csum;
	
	return csum;
}

// Use the BlockDataCheckSum() command to write a checksum value
static bool writeBlockChecksum(bq27441_dev_t *dev, uint8_t csum)
{
	return dev->write(BQ27441_EXTENDED_CHECKSUM, &csum, 1);	
}

// Read a byte from extended data specifying a class ID and position offset
static uint8_t readExtendedData(bq27441_dev_t *dev, uint8_t classID, uint8_t offset)
{
	uint8_t retData = 0;
	if (!_userConfigControl) BQ27441_enterConfig(dev, false);
		
	if (!blockDataControl(dev)) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(dev, classID)) // Write class ID using DataBlockClass()
		return false;
	
	blockDataOffset(dev, offset / 32); // Write 32-bit block offset (usually 0)
	
	computeBlockChecksum(dev); // Compute checksum going in
	//uint8_t oldCsum = blockDataChecksum(dev);
	/*for (int i=0; i<32; i++)
		Serial.print(String(readBlockData(i)) + " ");*/
	retData = readBlockData(dev, offset % 32); // Read from offset (limit to 0-31)
	
	if (!_userConfigControl) BQ27441_exitConfig(dev, true);
	
	return retData;
}

// Write a specified number of bytes to extended data specifying a 
// class ID, position offset.
static bool writeExtendedData(bq27441_dev_t *dev, uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len)
{
	if (len > 32)
		return false;
	
	if (!_userConfigControl) BQ27441_enterConfig(dev, false);
	
	if (!blockDataControl(dev)) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(dev, classID)) // Write class ID using DataBlockClass()
		return false;
	
	blockDataOffset(dev, offset / 32); // Write 32-bit block offset (usually 0)
	computeBlockChecksum(dev); // Compute checksum going in
	//uint8_t oldCsum = blockDataChecksum(dev);

	// Write data bytes:
	for (int i = 0; i < len; i++)
	{
		// Write to offset, mod 32 if offset is greater than 32
		// The blockDataOffset above sets the 32-bit block
		writeBlockData(dev, (offset % 32) + i, data[i]);
	}
	
	// Write new checksum using BlockDataChecksum (0x60)
	uint8_t newCsum = computeBlockChecksum(dev); // Compute the new checksum
	writeBlockChecksum(dev, newCsum);

	if (!_userConfigControl) BQ27441_exitConfig(dev, true);
	
	return true;
}

static int32_t constrain(int32_t x, int32_t a, int32_t b)
{   
    if(x < a)
        x = a;
    
    if(a > b)
        x = b;

    return x;
}

/*****************************************************************************
 ************************ I2C Read and Write Routines ************************
 *****************************************************************************/

// // Read a specified number of bytes over I2C at a given subAddress
// int16_t BQ27441_i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
// {
// 	int16_t timeout = BQ72441_I2C_TIMEOUT;	
// 	Wire.beginTransmission(_deviceAddress);
// 	Wire.write(subAddress);
// 	Wire.endTransmission(true);
	
// 	Wire.requestFrom(_deviceAddress, count);
	
// 	for (int i=0; i<count; i++)
// 	{
// 		dest[i] = Wire.read();
// 	}
	
// 	return timeout;
// }

// // Write a specified number of bytes over I2C to a given subAddress
// uint16_t BQ27441_i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count)
// {
// 	Wire.beginTransmission(_deviceAddress);
// 	Wire.write(subAddress);
// 	for (int i=0; i<count; i++)
// 	{
// 		Wire.write(src[i]);
// 	}	
// 	Wire.endTransmission(true);
	
// 	return true;	
// }

// BQ27441 lipo; // Use lipo.[] to interact with the library in an Arduino sketch
