#include "../Inc/IMU/IMU.h"
#include "../Inc/IMU/usart.h"
#include "../Inc/IMU/i2c.h"
#include "../Inc/IMU/bno055.h"


/*
  GetAccelData(&hi2c1, (uint8_t*)imu_readings);
  accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);      // Turn the MSB and LSB into a signed 16-bit value
  accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
  accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
  acc_x = ((float)(accel_data[0]))/100.0f; //m/s2
  acc_y = ((float)(accel_data[1]))/100.0f;
  acc_z = ((float)(accel_data[2]))/100.0f;

  velocityX = lastVelocityX + (acc_x * 1);
  lastVelocityX = velocityX;
 */


uint8_t GPwrMode 	= NormalG;   		// Gyro power mode
uint8_t Gscale 		= GFS_2000DPS; 	// Gyro full scale
//uint8_t Godr	 	= GODR_250Hz;  	// Gyro sample rate
uint8_t Gbw 			= GBW_230Hz;    // Gyro bandwidth
//
uint8_t Ascale 		= AFS_16G;      // Accel full scale
uint8_t APwrMode 	= NormalA;   		// Accel power mode
uint8_t Abw 			= ABW_250Hz;    // Accel bandwidth, accel sample rate divided by ABW_divx
//
//uint8_t Mscale 	= MFS_4Gauss;		// Select magnetometer full-scale resolution
uint8_t MOpMode 	= EnhancedRegular;    	// Select magnetometer perfomance mode
uint8_t MPwrMode 	= Normal;    		// Select magnetometer power mode
uint8_t Modr 			= MODR_30Hz;    // Select magnetometer ODR when in BNO055 bypass mode

uint8_t PWRMode 	= Normalpwr;  	// Select BNO055 power mode
uint8_t OPRMode 	= ACCGYRO;    	// specify operation mode for sensors [ACCONLY|MAGONLY|GYROONLY|ACCMAG|ACCGYRO|MAGGYRO|AMG|NDOF|NDOF_FMC_OFF]

uint8_t status;               // BNO055 data status register
float aRes, gRes, mRes; 			// scale resolutions per LSB for the sensors

// IMU calibration variables
uint8_t cal_sys 	= 0;
uint8_t cal_gyro 	= 0;
uint8_t cal_acc 	= 0;
uint8_t cal_mag 	= 0;
uint8_t cal_imu 	= 0;

const uint8_t num_of_bytes_read = 18;		// Read number of bytes from IMU (24 for ACCGYRO; 38 for NDOF)

const char read_devid[] 	= {START_BYTE, REG_READ, BNO055_CHIP_ID, 0x01};
//const char read_acc[] 		= {REG_READ, BNO055_ACC_DATA_X_LSB, num_of_bytes_read};
const char read_calib[2] 	= {REG_READ, BNO055_CALIB_STAT};
const char reset_sensor[3]	= {REG_WRITE, BNO055_SYS_TRIGGER, 0x01 << 5};
uint8_t get_readings[1] 	= {BNO055_ACC_DATA_X_LSB};

// Configure BNO sensor
void BNO055_Init_I2C(I2C_HandleTypeDef* hi2c_device) {
	// Select BNO055 config mode
	uint8_t opr_config_mode[2] = {BNO055_OPR_MODE, CONFIGMODE};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_config_mode, sizeof(opr_config_mode), 10);
	HAL_Delay(10);

	// Select page 1 to configure sensors
	uint8_t conf_page1[2] = {BNO055_PAGE_ID, 0x01};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page1, sizeof(conf_page1), 10);
	HAL_Delay(10);

	// Configure ACC (Page 1; 0x08)
	uint8_t conf_acc[2] = {BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_acc, sizeof(conf_acc), 10);
	HAL_Delay(10);

	// Configure GYR
	uint8_t conf_gyro[2] = {BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro, sizeof(conf_gyro), 10);
	HAL_Delay(10);

	uint8_t conf_gyro_pwr[2] = {BNO055_GYRO_CONFIG_1, GPwrMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro_pwr, sizeof(conf_gyro_pwr), 10);
	HAL_Delay(10);

	// Configure MAG
	uint8_t conf_mag_pwr[4] = {REG_WRITE, BNO055_MAG_CONFIG, 0x01, MPwrMode << 5 | MOpMode << 3 | Modr};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_mag_pwr, sizeof(conf_mag_pwr), 10);
	HAL_Delay(10);

	// Select BNO055 gyro temperature source
	//PutHexString(START_BYTE, BNO055_TEMP_SOURCE, 0x01 );

	// Select page 0
	uint8_t conf_page0[2] = {BNO055_PAGE_ID, 0x00};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page0, sizeof(conf_page0), 10);
	HAL_Delay(10);

	// Select BNO055 sensor units (Page 0; 0x3B, default value 0x80)
	/*- ORIENTATION_MODE		 - Android					(default)
		- VECTOR_ACCELEROMETER - m/s^2  					(default)
		- VECTOR_MAGNETOMETER  - uT								(default)
		- VECTOR_GYROSCOPE     - rad/s        v		(must be configured)
		- VECTOR_EULER         - degrees					(default)
		- VECTOR_LINEARACCEL   - m/s^2        v		(default)
		- VECTOR_GRAVITY       - m/s^2						(default)
	*/
	//const char conf_units[4] = {REG_WRITE, BNO055_UNIT_SEL, 0x01, 0x82};
	//SendAccelData(USART1, (uint8_t*)conf_units);
	//HAL_Delay(50);

	// Select BNO055 system power mode (Page 0; 0x3E)
	uint8_t pwr_pwrmode[2] = {BNO055_PWR_MODE, PWRMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, pwr_pwrmode, sizeof(pwr_pwrmode), 10);
	HAL_Delay(10);

	// Select BNO055 system operation mode (Page 0; 0x3D)
	uint8_t opr_oprmode[2] = {BNO055_OPR_MODE, OPRMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_oprmode, sizeof(opr_oprmode), 10);
	HAL_Delay(50);
}

int * getAcceleration(void) {
	//TIME IS INCLUDED IN THIS CALC

	static int *accelMag[2];
	int *accelXYZ = getTrueAccelerationXYZ();

	//Accel is mostly in Z
	if(accelXYZ[0] < 1 && accelXYZ[2] < 1 && accelXYZ[4] > 1) {
		accelMag[0] = (int)sqrt(pow(accelXYZ[0],2) + pow(accelXYZ[2],2) + pow(accelXYZ[4] - 9,2));
		accelMag[1] = (int)sqrt(pow(accelXYZ[1],2) + pow(accelXYZ[3],2) + pow(accelXYZ[5] - 8,2));
	}
	//Accel is mostly in Y
	else if (accelXYZ[0] < 1 && accelXYZ[2] > 1 && accelXYZ[4] < 1) {
		accelMag[0] = (int)sqrt(pow(accelXYZ[0],2) + pow(accelXYZ[2] - 9,2) + pow(accelXYZ[4],2));
		accelMag[1] = (int)sqrt(pow(accelXYZ[1],2) + pow(accelXYZ[3] - 8,2) + pow(accelXYZ[5],2));
	}
	//Accel is mostly in X
	else if (accelXYZ[0] > 1 && accelXYZ[2] < 1 && accelXYZ[4] < 1) {
		accelMag[0] = (int)sqrt(pow(accelXYZ[0] - 9,2) + pow(accelXYZ[2],2) + pow(accelXYZ[4],2));
		accelMag[1] = (int)sqrt(pow(accelXYZ[1] - 8,2) + pow(accelXYZ[3],2) + pow(accelXYZ[5],2));
	}

	//If all are greater than 1 (Defualt)
	if(accelXYZ[0] < 1 && accelXYZ[2] < 1 && accelXYZ[4] < 1){
		accelMag[0] = (int)sqrt(pow(accelXYZ[0],2) + pow(accelXYZ[2],2) + pow(accelXYZ[4],2));
		accelMag[1] = (int)sqrt(pow(accelXYZ[1],2) + pow(accelXYZ[3],2) + pow(accelXYZ[5],2));
	}

	return accelMag;
}


//TODO Convert to tinyINT(int16_t) or uint_16
int * getTrueAccelerationXYZ(void) {

	uint8_t	imu_readings[IMU_NUMBER_OF_BYTES];
	int16_t accel_data[3];

	static int accelXYZ[6];

	//Get All Accel Data and Parse into an array
    GetAccelData(&hi2c1, (uint8_t*)imu_readings);

	accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);
	accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
	accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);

	float acc_x = ((float)(accel_data[0]))/100.0f; // m per s^2
	float acc_y = ((float)(accel_data[1]))/100.0f; // m per s^2
	float acc_z = ((float)(accel_data[2]))/100.0f; // m per s^2

	accelXYZ[0] = (int)(acc_x);
	accelXYZ[1] = abs((int)((acc_x - (int)acc_x)*100));
	accelXYZ[2] = (int)(acc_y);
	accelXYZ[3] = abs((int)((acc_y - (int)acc_y)*100));
	accelXYZ[4] = (int)(acc_z);
	accelXYZ[5] = abs((int)((acc_z - (int)acc_z)*100));

	return accelXYZ;
}

int * getQuats(void){

//	uint8_t	imu_readings[IMU_NUMBER_OF_BYTES];
//	GetQuatData(&hi2c1, (uint8_t*)imu_readings);

	//https://github.com/adafruit/Adafruit_BNO055/blob/3f66fd8d0aa57ee8f487498d6c4de837c62bd7e2/Adafruit_BNO055.cpp
	static int quats[3] = {1,2,3};

	return quats;
}

// Send data to BNO055 over I2C
uint8_t GetQuatData(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
	return status;
}


// Send data to BNO055 over I2C
uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return status;
}

// TBD
uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id) {
	return HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT, chip_id, 1, 100);
}


// TBD
uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_device) {
	uint8_t temp;
	HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_TEMP, I2C_MEMADD_SIZE_8BIT, &temp, 1);
	while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return temp;
}


// Get IMU calibration values
uint8_t BNO055_Get_Calibration(I2C_HandleTypeDef* hi2c_device) {
	uint8_t calibration;
	HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT, &calibration, 1);
	while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY);
	return calibration;
}


// Calculate IMU calibration values
void BNO055_Calc_Calibration(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro, uint8_t *cal_acc, uint8_t *cal_mag) {
	*cal_system = (calibration >> 6) & 0x03;
	*cal_gyro 	= (calibration >> 4) & 0x03;
	*cal_acc 		= (calibration >> 2) & 0x03;
	*cal_mag 		= (calibration) & 0x03;
}
