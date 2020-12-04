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

#include "../Inc/SSD1306/ssd1306.h"
#include "../Inc/SSD1306/ssd1306_tests.h"
#include "../Inc/GPS/usart.h"
#include "../Inc/GPS/GPS.h"
#include "../Inc/IMU/IMU.h"
#include "../Inc/BB.h"
#include "../Inc/LoRa.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define BB_I2C_ADDRESS 0x55
#define BB_COMMAND_REM_CAPACITY	0x0C
#define BQ72441_I2C_TIMEOUT 2000

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

int state;
uint8_t data[512] = {0};
int idx = 0;
int numBytes = 0;
bool inSession = false;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void SPI1_Init(void);

void USART2_IRQHandler(void);

void displayHomeScreen(void);
void zeroStats(void);
int sessionStart(void);
void session(void);
void sessionNoSpeed(void);
void sessionRadio(void);

int main(void) {
	HAL_Init();

    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
	MX_I2C2_Init();
	MX_I2C3_Init();
	MX_GPIO_Init_USART();
	MX_USART2_UART_Init();

	state = 1;

    if(DEV) {
    	SPI1_Init();
    	LoRa_Init();
    	uint8_t pack[] = "Snow-weAR";
    	loraTransmit(pack, 9);
		HAL_NVIC_SetPriority(RXDone_EXTI_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(RXDone_EXTI_IRQn);
		loraReceiveModeInit();
    }

    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();

    BNO055_Init_I2C(&hi2c1);

    while(1) {
    	if(state == 1) {
    		displayHomeScreen();
    	} else if(state == 2) {
    		session();
    	} else if(state == 3){
    		sessionNoSpeed();
    	} else if(state == 4){
    		sessionRadio();
    	}
    }
}

void USART2_IRQHandler(void) {
	if(USART2->ISR & USART_ISR_RXNE) {
		char rx = (char)(USART2->RDR & 0xFF);
		data[idx++] = rx;

		if(rx == '\n') {
			numBytes++;
			if(data[3] == 71 && data[4] == 71) {
				memcpy((void*)GPS.rxBuffer, data, idx);
				GPS_Parse();
			}
			else if(data[3] == 86 && data[4] == 84) {
				memcpy((void*)GPS.rxBuffer, data, idx);
				GPS_Parse();
			}
			idx = 0;
		}
	}
}

void displayHomeScreen(void) {
	int logo[739][2] = {{11, 19}, {11, 20}, {11, 21}, {11, 22}, {11, 23}, {11, 24}, {11, 25}, {11, 26}, {11, 27}, {11, 28}, {11, 29}, {11, 30}, {12, 13}, {12, 14}, {12, 15}, {12, 16}, {12, 17}, {12, 18}, {12, 19}, {12, 20}, {12, 21}, {12, 22}, {12, 23}, {12, 24}, {12, 25}, {12, 26}, {12, 27}, {12, 28}, {12, 29}, {12, 30}, {12, 31}, {12, 32}, {12, 33}, {12, 34}, {12, 35}, {12, 36}, {13, 9}, {13, 10}, {13, 11}, {13, 12}, {13, 13}, {13, 14}, {13, 15}, {13, 16}, {13, 17}, {13, 18}, {13, 19}, {13, 20}, {13, 21}, {13, 22}, {13, 23}, {13, 24}, {13, 25}, {13, 26}, {13, 27}, {13, 28}, {13, 29}, {13, 30}, {13, 31}, {13, 32}, {13, 33}, {13, 34}, {13, 35}, {13, 36}, {13, 37}, {13, 38}, {13, 39}, {13, 40}, {14, 7}, {14, 8}, {14, 9}, {14, 10}, {14, 11}, {14, 12}, {14, 13}, {14, 14}, {14, 15}, {14, 16}, {14, 17}, {14, 18}, {14, 19}, {14, 20}, {14, 21}, {14, 22}, {14, 23}, {14, 24}, {14, 25}, {14, 26}, {14, 27}, {14, 28}, {14, 29}, {14, 30}, {14, 31}, {14, 32}, {14, 33}, {14, 34}, {14, 35}, {14, 36}, {14, 37}, {14, 38}, {14, 39}, {14, 40}, {14, 41}, {14, 42}, {15, 6}, {15, 7}, {15, 8}, {15, 9}, {15, 10}, {15, 11}, {15, 12}, {15, 13}, {15, 14}, {15, 15}, {15, 16}, {15, 17}, {15, 18}, {15, 19}, {15, 20}, {15, 21}, {15, 22}, {15, 23}, {15, 24}, {15, 25}, {15, 26}, {15, 27}, {15, 28}, {15, 29}, {15, 30}, {15, 31}, {15, 32}, {15, 33}, {15, 34}, {15, 35}, {15, 36}, {15, 37}, {15, 38}, {15, 39}, {15, 40}, {15, 41}, {15, 42}, {15, 43}, {15, 44}, {16, 4}, {16, 5}, {16, 6}, {16, 7}, {16, 8}, {16, 9}, {16, 10}, {16, 11}, {16, 12}, {16, 13}, {16, 14}, {16, 15}, {16, 16}, {16, 17}, {16, 32}, {16, 33}, {16, 34}, {16, 35}, {16, 36}, {16, 37}, {16, 38}, {16, 39}, {16, 40}, {16, 41}, {16, 42}, {16, 43}, {16, 44}, {16, 45}, {17, 3}, {17, 4}, {17, 5}, {17, 6}, {17, 7}, {17, 8}, {17, 9}, {17, 10}, {17, 11}, {17, 38}, {17, 39}, {17, 40}, {17, 41}, {17, 42}, {17, 43}, {17, 44}, {17, 45}, {17, 46}, {18, 3}, {18, 4}, {18, 5}, {18, 6}, {18, 7}, {18, 8}, {18, 41}, {18, 42}, {18, 43}, {18, 44}, {18, 45}, {18, 46}, {19, 2}, {19, 3}, {19, 4}, {19, 5}, {19, 6}, {19, 7}, {19, 43}, {19, 44}, {19, 45}, {19, 46}, {19, 47}, {20, 2}, {20, 3}, {20, 4}, {20, 5}, {20, 6}, {20, 43}, {20, 44}, {20, 45}, {20, 46}, {20, 47}, {21, 1}, {21, 2}, {21, 3}, {21, 4}, {21, 5}, {21, 24}, {21, 25}, {21, 44}, {21, 45}, {21, 46}, {21, 47}, {21, 48}, {21, 49}, {22, 0}, {22, 1}, {22, 2}, {22, 3}, {22, 4}, {22, 5}, {22, 22}, {22, 25}, {22, 26}, {22, 27}, {22, 44}, {22, 46}, {22, 47}, {22, 48}, {22, 49}, {23, 0}, {23, 1}, {23, 2}, {23, 3}, {23, 4}, {23, 5}, {23, 20}, {23, 26}, {23, 27}, {23, 28}, {23, 44}, {23, 45}, {23, 46}, {23, 47}, {23, 48}, {23, 49}, {24, 0}, {24, 1}, {24, 2}, {24, 3}, {24, 4}, {24, 5}, {24, 15}, {24, 16}, {24, 17}, {24, 18}, {24, 19}, {24, 20}, {24, 23}, {24, 24}, {24, 25}, {24, 26}, {24, 27}, {24, 28}, {24, 29}, {24, 30}, {24, 31}, {24, 32}, {24, 44}, {24, 45}, {24, 46}, {24, 47}, {24, 48}, {24, 49}, {25, 0}, {25, 1}, {25, 2}, {25, 3}, {25, 4}, {25, 5}, {25, 14}, {25, 17}, {25, 18}, {25, 19}, {25, 20}, {25, 21}, {25, 22}, {25, 23}, {25, 24}, {25, 25}, {25, 26}, {25, 27}, {25, 28}, {25, 29}, {25, 30}, {25, 31}, {25, 32}, {25, 33}, {25, 34}, {25, 44}, {25, 46}, {25, 47}, {25, 48}, {25, 49}, {26, 0}, {26, 1}, {26, 2}, {26, 3}, {26, 4}, {26, 5}, {26, 13}, {26, 18}, {26, 19}, {26, 20}, {26, 21}, {26, 22}, {26, 23}, {26, 24}, {26, 25}, {26, 26}, {26, 27}, {26, 28}, {26, 29}, {26, 30}, {26, 31}, {26, 32}, {26, 33}, {26, 34}, {26, 35}, {26, 39}, {26, 40}, {26, 41}, {26, 44}, {26, 45}, {26, 46}, {26, 47}, {26, 48}, {26, 49}, {27, 0}, {27, 1}, {27, 2}, {27, 3}, {27, 4}, {27, 5}, {27, 12}, {27, 17}, {27, 18}, {27, 19}, {27, 20}, {27, 21}, {27, 28}, {27, 29}, {27, 30}, {27, 31}, {27, 32}, {27, 33}, {27, 34}, {27, 35}, {27, 36}, {27, 37}, {27, 38}, {27, 42}, {27, 43}, {27, 44}, {27, 46}, {27, 47}, {27, 48}, {27, 49}, {28, 0}, {28, 1}, {28, 2}, {28, 3}, {28, 5}, {28, 8}, {28, 9}, {28, 10}, {28, 11}, {28, 12}, {28, 13}, {28, 14}, {28, 15}, {28, 16}, {28, 17}, {28, 18}, {28, 19}, {28, 20}, {28, 22}, {28, 23}, {28, 24}, {28, 25}, {28, 26}, {28, 27}, {28, 29}, {28, 30}, {28, 31}, {28, 32}, {28, 33}, {28, 34}, {28, 35}, {28, 36}, {28, 37}, {28, 38}, {28, 39}, {28, 40}, {28, 42}, {28, 43}, {28, 44}, {28, 46}, {28, 47}, {28, 48}, {28, 49}, {29, 0}, {29, 1}, {29, 2}, {29, 3}, {29, 5}, {29, 6}, {29, 7}, {29, 10}, {29, 11}, {29, 12}, {29, 13}, {29, 14}, {29, 15}, {29, 16}, {29, 17}, {29, 18}, {29, 19}, {29, 21}, {29, 22}, {29, 23}, {29, 24}, {29, 25}, {29, 26}, {29, 27}, {29, 28}, {29, 30}, {29, 31}, {29, 32}, {29, 33}, {29, 34}, {29, 35}, {29, 36}, {29, 37}, {29, 38}, {29, 39}, {29, 40}, {29, 41}, {29, 42}, {29, 43}, {29, 44}, {29, 45}, {29, 46}, {29, 47}, {29, 48}, {29, 49}, {30, 0}, {30, 1}, {30, 2}, {30, 3}, {30, 4}, {30, 5}, {30, 6}, {30, 7}, {30, 8}, {30, 9}, {30, 10}, {30, 11}, {30, 12}, {30, 13}, {30, 14}, {30, 15}, {30, 16}, {30, 17}, {30, 18}, {30, 20}, {30, 21}, {30, 22}, {30, 23}, {30, 24}, {30, 25}, {30, 26}, {30, 27}, {30, 28}, {30, 29}, {30, 31}, {30, 32}, {30, 33}, {30, 34}, {30, 35}, {30, 36}, {30, 37}, {30, 38}, {30, 39}, {30, 40}, {30, 41}, {30, 42}, {30, 43}, {30, 44}, {30, 46}, {30, 47}, {30, 48}, {30, 49}, {31, 0}, {31, 1}, {31, 2}, {31, 3}, {31, 4}, {31, 5}, {31, 6}, {31, 7}, {31, 8}, {31, 9}, {31, 10}, {31, 11}, {31, 12}, {31, 13}, {31, 14}, {31, 15}, {31, 16}, {31, 17}, {31, 20}, {31, 21}, {31, 22}, {31, 23}, {31, 24}, {31, 25}, {31, 26}, {31, 27}, {31, 28}, {31, 29}, {31, 32}, {31, 33}, {31, 34}, {31, 35}, {31, 36}, {31, 37}, {31, 38}, {31, 39}, {31, 40}, {31, 41}, {31, 42}, {31, 43}, {31, 45}, {31, 46}, {31, 47}, {31, 48}, {31, 49}, {32, 0}, {32, 1}, {32, 2}, {32, 3}, {32, 4}, {32, 5}, {32, 7}, {32, 8}, {32, 9}, {32, 10}, {32, 11}, {32, 12}, {32, 13}, {32, 14}, {32, 15}, {32, 16}, {32, 17}, {32, 19}, {32, 20}, {32, 21}, {32, 22}, {32, 23}, {32, 26}, {32, 27}, {32, 28}, {32, 29}, {32, 30}, {32, 32}, {32, 33}, {32, 34}, {32, 35}, {32, 36}, {32, 37}, {32, 38}, {32, 39}, {32, 40}, {32, 41}, {32, 42}, {32, 44}, {32, 45}, {32, 46}, {32, 47}, {32, 48}, {32, 49}, {33, 2}, {33, 3}, {33, 4}, {33, 5}, {33, 6}, {33, 7}, {33, 9}, {33, 10}, {33, 11}, {33, 12}, {33, 13}, {33, 14}, {33, 15}, {33, 16}, {33, 18}, {33, 19}, {33, 20}, {33, 21}, {33, 22}, {33, 27}, {33, 28}, {33, 29}, {33, 30}, {33, 31}, {33, 33}, {33, 34}, {33, 35}, {33, 36}, {33, 37}, {33, 38}, {33, 39}, {33, 40}, {33, 42}, {33, 43}, {33, 44}, {33, 45}, {33, 46}, {33, 47}, {34, 5}, {34, 6}, {34, 7}, {34, 8}, {34, 9}, {34, 12}, {34, 13}, {34, 14}, {34, 15}, {34, 17}, {34, 18}, {34, 19}, {34, 20}, {34, 21}, {34, 28}, {34, 29}, {34, 30}, {34, 31}, {34, 32}, {34, 34}, {34, 35}, {34, 36}, {34, 37}, {34, 40}, {34, 41}, {34, 42}, {34, 43}, {34, 44}, {35, 7}, {35, 8}, {35, 9}, {35, 10}, {35, 11}, {35, 12}, {35, 16}, {35, 17}, {35, 18}, {35, 19}, {35, 20}, {35, 29}, {35, 30}, {35, 31}, {35, 32}, {35, 33}, {35, 34}, {35, 37}, {35, 38}, {35, 39}, {35, 40}, {35, 41}, {35, 42}, {36, 9}, {36, 10}, {36, 11}, {36, 12}, {36, 13}, {36, 14}, {36, 15}, {36, 16}, {36, 17}, {36, 18}, {36, 19}, {36, 30}, {36, 31}, {36, 32}, {36, 33}, {36, 34}, {36, 35}, {36, 36}, {36, 37}, {36, 38}, {36, 39}, {36, 40}, {37, 12}, {37, 13}, {37, 14}, {37, 15}, {37, 16}, {37, 17}, {37, 18}, {37, 19}, {37, 30}, {37, 31}, {37, 32}, {37, 33}, {37, 34}, {37, 35}, {37, 36}, {37, 37}};
	ssd1306_Fill(Black);
	int logo_idx;
	for(logo_idx = 0; logo_idx < 739; logo_idx++) {
		int x = logo[logo_idx][1] + 40;
		int y = logo[logo_idx][0];
		ssd1306_DrawPixel(x, y, White);
	}
	ssd1306_UpdateScreen();
	char snow[17] = "Snow-weAR Goggles";
	ssd1306_SetCursor(4, 45);
	ssd1306_WriteString("Snow-weAR Goggles", Font_7x10, White);
	ssd1306_UpdateScreen();
	state = -1;
}

void session(void) {
	ssd1306_Fill(Black);

	int cur_soc = -1;
	int cur_temp = -1;

	//IMU
	int Vmag[2] = {0,0};
	int Vmag_max[2] = {0,0};
	int vxd,vyd,vzd;

	float V[3] = {0.0,0.0,0.0};
	float V_last[3] = {0.0,0.0,0.0};
	float accel[3] = {0.0,0.0,0.0};

    //IMU Testing
    int * pureAccel;

	while(1) {
		if(state == 1 || state == 3 || state == 4)
			return;
		uint16_t soc = BB_soc(&hi2c1, 0);
		if(soc != cur_soc) {
			cur_soc = soc;
			ssd1306_SetCursor(100, 54);
			char soc_str[30];
			sprintf(soc_str, "%3d%% ", soc);
			ssd1306_WriteString(soc_str, Font_6x8, White);
		}

		uint16_t temp = readTemp(&hi2c1);
		if(temp != cur_temp) {
			cur_temp = temp;
			char temp_str[15];
			sprintf(temp_str, "%2d.%1d F",(int)temp/10, (int)temp%10);
			ssd1306_SetCursor(89, 10);
			ssd1306_WriteString(temp_str, Font_6x8, White);
			ssd1306_DrawCircle(115, 11, 1, White);
		}


		//IMU Velocity Mag (m/s)
		getVelocity(V, V_last,accel);
		char v_str[30];
		sprintf(v_str, "V(m/s): %d.%d",(int)Vmag[0],abs((int)((Vmag[0]-(int)Vmag[0])*100)));
		ssd1306_SetCursor(4, 20);
		ssd1306_WriteString(v_str, Font_6x8, White);

		//Max Velocity (m/s)
		if(Vmag[0] > Vmag_max[0]){
			if(Vmag[1] > Vmag_max[1]){
				Vmag_max[0] = Vmag[0];
				Vmag_max[1] = abs((int)((Vmag[0]-(int)Vmag[0])*100));
			}
		}

		char vmx_str[30];
		sprintf(vmx_str, "Vmax(m/s): %d.%d",Vmag_max[0],Vmag_max[1]);
		ssd1306_SetCursor(4, 30);
		ssd1306_WriteString(vmx_str, Font_6x8, White);

		//Individual Velocity
		vxd = abs((int)((V_last[0]-(int)V_last[0])*100));
		vyd = abs((int)((V_last[1]-(int)V_last[1])*100));
		vzd = abs((int)((V_last[2]-(int)V_last[2])*100));

		char vs_str[30];
		sprintf(vs_str, "Vx:%d.%2d,Vy:%d.%2d,Vz:%d.%2d",(int)V_last[0],vxd,(int)V_last[1],vyd,(int)V_last[2],vzd);
		ssd1306_SetCursor(4, 40);
		ssd1306_WriteString(vs_str, Font_6x8, White);


		// Read GPS
//		char lat_str[15];
//		char lon_str[15];
//		sprintf(lat_str, "%2d.%2d%c", (int)GPS.LatDec, (int)((GPS.LatDec-(int)GPS.LatDec)*100), GPS.NS_Indicator);
//		ssd1306_SetCursor(30, 40);
//		ssd1306_WriteString(lat_str, Font_6x8, White);
//		ssd1306_SetCursor(72, 40);
//		sprintf(lon_str, "%2d.%2d%c", (int)GPS.LonDec, (int)((GPS.LonDec-(int)GPS.LonDec)*100), GPS.EW_Indicator);
//		ssd1306_WriteString(lon_str, Font_6x8, White);
//		ssd1306_SetCursor(4, 40);
//		char gps_msg[10];
//		sprintf(gps_msg, "GPS:");
//		ssd1306_WriteString(gps_msg, Font_6x8, White);
//
//		ssd1306_SetCursor(4, 54);
//		ssd1306_WriteString(gpsStringData, Font_6x8, White);

		// Update Screen
		ssd1306_UpdateScreen();
	}
	inSession = false;
	return;
}

void sessionNoSpeed(void) {
	ssd1306_Fill(Black);

	int cur_soc = -1;
	int cur_temp = -1;

	while(1) {
		if(state == 1 || state == 2 || state == 4)
			return;
		uint16_t soc = BB_soc(&hi2c1, 0);
		if(soc != cur_soc) {
			cur_soc = soc;
			ssd1306_SetCursor(100, 54);
			char soc_str[30];
			sprintf(soc_str, "%3d%% ", soc);
			ssd1306_WriteString(soc_str, Font_6x8, White);
		}

		uint16_t temp = readTemp(&hi2c1);
		if(temp != cur_temp) {
			cur_temp = temp;
			char temp_str[15];
			sprintf(temp_str, "%2d.%1d F",(int)temp/10, (int)temp%10);
			ssd1306_SetCursor(89, 10);
			ssd1306_WriteString(temp_str, Font_6x8, White);
			ssd1306_DrawCircle(115, 11, 1, White);
		}

		// Read GPS
		char lat_str[15];
		char lon_str[15];
		sprintf(lat_str, "%2d.%2d%c", (int)GPS.LatDec, (int)((GPS.LatDec-(int)GPS.LatDec)*100), GPS.NS_Indicator);
		ssd1306_SetCursor(30, 30);
		ssd1306_WriteString(lat_str, Font_6x8, White);
		ssd1306_SetCursor(72, 30);
		sprintf(lon_str, "%2d.%2d%c", (int)GPS.LonDec, (int)((GPS.LonDec-(int)GPS.LonDec)*100), GPS.EW_Indicator);
		ssd1306_WriteString(lon_str, Font_6x8, White);
		ssd1306_SetCursor(4, 30);
		char gps_msg[10];
		sprintf(gps_msg, "GPS:");
		ssd1306_WriteString(gps_msg, Font_6x8, White);

		// Update Screen
		ssd1306_UpdateScreen();
	}
	inSession = false;
	return;
}

void sessionRadio(void) {
	ssd1306_Fill(Black);
	HAL_NVIC_EnableIRQ(RXDone_EXTI_IRQn);

	int cur_soc = -1;
	int cur_temp = -1;

	while(1) {
		if(state == 1 || state == 2 || state == 3)
			return;
		uint16_t soc = BB_soc(&hi2c1, 0);
		if(soc != cur_soc) {
			cur_soc = soc;
			ssd1306_SetCursor(100, 54);
			char soc_str[30];
			sprintf(soc_str, "%3d%% ", soc);
			ssd1306_WriteString(soc_str, Font_6x8, White);
		}

		uint16_t temp = readTemp(&hi2c1);
		if(temp != cur_temp) {
			cur_temp = temp;
			char temp_str[15];
			sprintf(temp_str, "%2d.%1d F",(int)temp/10, (int)temp%10);
			ssd1306_SetCursor(89, 10);
			ssd1306_WriteString(temp_str, Font_6x8, White);
			ssd1306_DrawCircle(115, 11, 1, White);
		}

		//Display Radio Data
		ssd1306_SetCursor(4, 30);
		char write_this[25];
		sprintf(write_this, "GPS data from LoRa:");
		ssd1306_WriteString(write_this, Font_6x8, White);
		ssd1306_SetCursor(4, 40);
		ssd1306_WriteString(gpsStringData, Font_6x8, White);

		// Update Screen
		ssd1306_UpdateScreen();
	}
	return;
}


int sessionStart(void) {
	int ss_ok = 0;
	ssd1306_Fill(Black);
	ssd1306_SetCursor(25, 25);
	ssd1306_WriteString("Session Start", Font_7x10, White);
	ssd1306_UpdateScreen();

	ssd1306_SetCursor(25, 40);
	BB_Init();
	bool check;
	check = BB_begin(&hi2c1);
	if(check) {
		ssd1306_WriteString("*", Font_7x10, White);
		ssd1306_UpdateScreen();
	}
	else {
		ssd1306_WriteString(" ", Font_7x10, White);
		ssd1306_UpdateScreen();
		ss_ok = 1;
	}
/*
	ssd1306_SetCursor(40, 40);
	if(imu_ok == HAL_OK) {
		ssd1306_WriteString("*", Font_7x10, White);
		ssd1306_UpdateScreen();
	}
	else {
		ssd1306_WriteString(" ", Font_7x10, White);
		ssd1306_UpdateScreen();
	}
	ss_ok += imu_ok;
*/
	return 1;
}

void zeroStats(void) {
	ssd1306_Fill(Black);
	ssd1306_SetCursor(4, 10);
	ssd1306_WriteString("00.0", Font_11x18, White);
	ssd1306_WriteString("kmh", Font_6x8, White);

	ssd1306_SetCursor(83, 15);
	char temp_str[15];
	sprintf(temp_str, "%02d.%1d F", 0, 0);
	ssd1306_WriteString(temp_str, Font_7x10, White);
	ssd1306_DrawCircle(115, 15, 1, White);


	ssd1306_DrawCircle(5, 54, 2, White);
	ssd1306_DrawArc(5, 60, 4, 90, 270, White);
	ssd1306_SetCursor(14, 54);
	char gps_str[20];
	sprintf(gps_str, "%02dN %02dW %02dU", 0, 0, 0);
	ssd1306_WriteString(gps_str, Font_6x8, White);

	ssd1306_SetCursor(100, 54);
	uint16_t soc = BB_soc(&hi2c1, 0);
	char soc_str[30];
	sprintf(soc_str, "%3d%% ", soc);
	ssd1306_WriteString(soc_str, Font_6x8, White);

	ssd1306_UpdateScreen();
	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  /* EXTI line interrupt detected */
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  if(GPIO_Pin == session_btn_Pin){
	  int session_btn_val = HAL_GPIO_ReadPin(session_btn_GPIO_Port, session_btn_Pin);

	  if(session_btn_val){
		  //Toggle Screen
		  if(state == 2){
			//Button Pressed Second Time -> Go to State 3
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			state = 3;
		  }
		  else if(state == 3) {
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			state = 4;
		  }
		  else if(state == 4) {
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			HAL_NVIC_DisableIRQ(RXDone_EXTI_IRQn);
			state = 1;
		  }
		  else {
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			state = 2; //Home
		  }

		  //Send GPS Data
		  uint8_t pack[] = "TEST";
		  loraTransmit(pack,4);
	  }

  }
  if(GPIO_Pin == RXDone_Pin){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	//HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	int irq_flags = readReg(RH_RF95_REG_12_IRQ_FLAGS);
	int num = readReg(RH_RF95_REG_13_RX_NB_BYTES);
	readReg(0x42);
	writeReg(RH_RF95_REG_12_IRQ_FLAGS, RH_RF95_RX_DONE_MASK);
	uint8_t buf[15];
	loraReceive(buf);
	for(int i = 0; i < 15; i++){
		gpsStringData[i] = (char)buf[i];
	}
	loraReceiveModeInit();
  }
  if(GPIO_Pin == radio_btn_Pin) {

	  GPS_String();
	  loraTransmit(GPS.str, 11);
  }
  return;
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  if(DEV){
	HAL_GPIO_WritePin(LORA_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LORA_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RXDone_GPIO_Port, RXDone_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin : LORA_NSS_Pin */
	GPIO_InitStruct.Pin = LORA_NSS_Pin | LORA_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LORA_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LORA_MOSI_Pin | LORA_MISO_Pin | LORA_SCLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Alternate = 0x05;
	HAL_GPIO_Init(LORA_SPI_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Interrupt test pin */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : RXDone_Pin */
	GPIO_InitStruct.Pin = RXDone_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RXDone_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(LORA_EXTI_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(LORA_EXTI_IRQn);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	//HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 1);
	//HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, BB_GPOUT_Pin|BB_CE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, BB_SYSOFF_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPSReg_Pwr_GPIO_Port, GPSReg_Pwr_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : BB_GPOUT_Pin BB_CE_Pin */
	GPIO_InitStruct.Pin = BB_GPOUT_Pin|BB_CE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : BB_SYSOFF_Pin CS_LoRa_Pin */
	GPIO_InitStruct.Pin = BB_SYSOFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : session_btn_Pin */
	GPIO_InitStruct.Pin = session_btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(session_btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : radio_btn_Pin */
	GPIO_InitStruct.Pin = radio_btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(radio_btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPSReg_ERR_Pin */
	GPIO_InitStruct.Pin = GPSReg_ERR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPSReg_ERR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPSReg_Pwr_Pin */
	GPIO_InitStruct.Pin = GPSReg_Pwr_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPSReg_Pwr_GPIO_Port, &GPIO_InitStruct);

 }else{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, IMU_ADDR_Pin|IMU_INT_Pin|IMU_nRST_Pin|IMU_BL_IND_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : IMU_ADDR_Pin IMU_INT_Pin IMU_nRST_Pin IMU_BL_IND_Pin */
	GPIO_InitStruct.Pin = IMU_ADDR_Pin|IMU_INT_Pin|IMU_nRST_Pin|IMU_BL_IND_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10808DD3;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
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

	NVIC_SetPriority(SPI1_IRQn, 2); /* (4) */
	NVIC_EnableIRQ(SPI1_IRQn); /* (5) */
}

static void MX_I2C2_Init(void) {
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10808DD3;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C3_Init(void) {
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10808DD3;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
    Error_Handler();
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void) { }

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
