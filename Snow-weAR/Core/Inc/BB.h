#include <stdbool.h>
#include <stdint.h>
#include <main.h>

static uint8_t _deviceAddress;  // Stores the BQ27441-G1A's I2C address
static bool _sealFlag; // Global to identify that IC was previously sealed
static bool _userConfigControl; // Global to identify that user has control over
                            // entering/exiting config

#define HAL_MAX_DELAY      0xFFFFFFFFU
#define BQ72441_I2C_TIMEOUT 2000

#ifndef BB_I2C_ADDR
#define BB_I2C_ADDR        0x55 << 1
#endif

typedef enum {
	REMAIN,     // Remaining Capacity (DEFAULT)
	FULL,       // Full Capacity
	AVAIL,      // Available Capacity
	AVAIL_FULL, // Full Available Capacity
	REMAIN_F,   // Remaining Capacity Filtered
	REMAIN_UF,  // Remaining Capacity Unfiltered
	FULL_F,     // Full Capacity Filtered
	FULL_UF,    // Full Capacity Unfiltered
	DESIGN      // Design Capacity
} capacity_measure;

// Parameters for the soc() function
typedef enum {
	FILTERED,  // State of Charge Filtered (DEFAULT)
	UNFILTERED // State of Charge Unfiltered
} soc_measure;

typedef enum {
	AVG,  // Average Current (DEFAULT)
	STBY, // Standby Current
	MAX   // Max Current
} current_measure;

typedef enum {
	PERCENT,  // State of Health Percentage (DEFAULT)
	SOH_STAT  // State of Health Status Bits
} soh_measure;

typedef enum {
	SOC_INT, // Set GPOUT to SOC_INT functionality
	BAT_LOW  // Set GPOUT to BAT_LOW functionality
} gpout_function;

void BB_Init(void);
bool BB_begin(I2C_HandleTypeDef*);
uint16_t readVoltage(I2C_HandleTypeDef*);
uint16_t readCapacity(I2C_HandleTypeDef*, capacity_measure);
int16_t readAvgPower(I2C_HandleTypeDef*);
uint16_t readTemp(I2C_HandleTypeDef*);
uint16_t BB_deviceType(I2C_HandleTypeDef*);
uint16_t readControlWord(uint16_t, I2C_HandleTypeDef*);
uint16_t readWord(uint16_t, I2C_HandleTypeDef*);
static bool executeControlWord(uint16_t, I2C_HandleTypeDef*);
bool setCap(I2C_HandleTypeDef*, uint16_t);
static bool writeExtendedData(I2C_HandleTypeDef*, uint8_t, uint8_t, uint8_t*, uint8_t);
static uint8_t readExtendedData(I2C_HandleTypeDef*, uint8_t, uint8_t);
bool BB_enterConfig(I2C_HandleTypeDef*, bool);
static bool sealed(I2C_HandleTypeDef*);
static bool seal(I2C_HandleTypeDef*);
static bool unseal(I2C_HandleTypeDef*);
uint16_t BB_status(I2C_HandleTypeDef*);
uint16_t BB_flags(I2C_HandleTypeDef*);
bool BB_exitConfig(I2C_HandleTypeDef*, bool);
static bool softReset(I2C_HandleTypeDef*);
static bool blockDataControl(I2C_HandleTypeDef*);
static bool blockDataClass(I2C_HandleTypeDef*, uint8_t);
static bool blockDataOffset(I2C_HandleTypeDef*, uint8_t);
static uint8_t blockDataChecksum(I2C_HandleTypeDef*);
static uint8_t computeBlockChecksum(I2C_HandleTypeDef*);
static bool writeBlockChecksum(I2C_HandleTypeDef*, uint8_t);
static bool writeBlockData(I2C_HandleTypeDef*, uint8_t, uint8_t);
uint16_t BB_soc(I2C_HandleTypeDef*, soc_measure);
int16_t BB_current(I2C_HandleTypeDef*, current_measure);
uint8_t BB_soh(I2C_HandleTypeDef*, soh_measure);
bool BB_GPOUTPolarity(I2C_HandleTypeDef*);
bool BB_setGPOUTPolarity(I2C_HandleTypeDef*, bool);
bool BB_GPOUTFunction(I2C_HandleTypeDef*);
bool BB_setGPOUTFunction(I2C_HandleTypeDef*, gpout_function);
uint8_t BB_SOC1SetThreshold(I2C_HandleTypeDef*);
uint8_t BB_SOC1ClearThreshold(I2C_HandleTypeDef*);
bool BB_setSOC1Thresholds(I2C_HandleTypeDef*, uint8_t, uint8_t);
uint8_t BB_SOCFSetThreshold(I2C_HandleTypeDef*);
uint8_t BB_SOCFClearThreshold(I2C_HandleTypeDef*);
bool BB_setSOCFThresholds(I2C_HandleTypeDef*, uint8_t, uint8_t);
bool BB_socFlag(I2C_HandleTypeDef*);
bool BB_socfFlag(I2C_HandleTypeDef*);
uint8_t BB_sociDelta(I2C_HandleTypeDef*);
bool BB_setSOCIDelta(I2C_HandleTypeDef*, uint8_t);
bool BB_pulseGPOUT(I2C_HandleTypeDef*);
static int32_t constrain(int32_t, int32_t, int32_t);
