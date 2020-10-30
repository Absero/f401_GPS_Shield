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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_UART_BUFFER_SIZE 2000

#define LSM6DSL_ADDRESS 0xD7
#define LSM6DSL_CTRL3_C 0x12    // control register for incrementing
#define LSM6DSL_ACC_ON_REG 0x10 // acc mode (turns on acc)
#define LSM6DSL_CTRL6_C 0x15    // control register for acc modes
#define LSM6DSL_OUTX_L_G 0x22   // first register of gyr reading

//temperature and humidity variables
#define HTS221_ADDRESS 0xBC
#define HTS221_WRITING_TH 0xBE
#define HTS221_READING_TH 0xBF

//magnetometer variables
#define LSM303_MAGN_ADDRESS 0x33        // magnetometer sensor adress
#define LSM303_READING_MAGN 0x3D        // adress to read from magn sensor registers
#define LSM303_WRITING_MAGN 0x3C        // address to write to magn sensor registers
#define LSM303_MAGN_ON_MODE 0x60        // control register to turn on magnetometer
#define LSM303_OUTX_L_REG_M 0x68 | 0x80 // magnetometer XX YY ZZ

//ACC
#define LSM303_WRITING_ACC2 0x32 // address to write to acc registers
#define LSM303_READING_ACC2 0x33 // address to read from acc registers
#define LSM303_CTRL_REG1_A 0x20  // modes register
#define LSM303_REG_OUT_ACC2 0x28 // first register to output data

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

struct {
	uint8_t PPS :1;
	uint8_t timer11 :1;
	uint8_t GPSDataProcessed :1;
	uint8_t unused :6;
} gFlags;

uint8_t gGPS_UART_buffer[GPS_UART_BUFFER_SIZE];
uint8_t *pFullPeriodicPacket;

//acc and gyro
uint8_t reg_increment[1] = { 0x04 };	         // register increment
uint8_t acc_gyr_on_values[2] = { 0x50, 0x50 };
;
// turn on acc and gyr
uint8_t acc_gyr_modes[2] = { 0x10, 0x80 };     // low/normal modes for acc and gyr
struct {
	uint8_t data_acc_gyr[12];                    // data received from acc and gyr sensors
	int16_t acc_xx, acc_yy, acc_zz, gyr_xx, gyr_yy, gyr_zz;
} gAccGyro;

//temperature and humidity variables
uint8_t hum_temp_on[2] = { 0x20, 0x83 };  //turn on sensor + 12.5Hz, NOT one-shot
uint8_t HUMIDITY_OUT_L[1] = { 0x28 };
uint8_t data_temp_hum[4];
uint8_t who_am_I_reg[1] = { 0x0F };
uint8_t whoami[1];
int16_t unpacked_temp_hum[2], temp, hum;

//magnetometer variables
uint8_t magn_on[1] = { 0x1C };  // continuous/single measurement mode
uint8_t data_magn[6];
int16_t unpacked_magn[3], magn_x, magn_y, magn_z;

//ACC
uint8_t reg[1] = { 0x57 };  // normal mode on
uint8_t data_acc2[12];
int32_t unpacked_acc2[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

uint16_t getNewlineIndex(uint8_t *array, uint16_t size, uint16_t num);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// Paruosti pointeri dinaminiam masyvui, kad po to galima visada butu naudot realloc
	// 	kadangi tas masyvas visada bus reikalingas, taip nereikes atlaisvinti uzimtos vietos
	pFullPeriodicPacket = (uint8_t*) malloc(1);

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART6_UART_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim11);

	//LSM6DSL init
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDRESS, LSM6DSL_ACC_ON_REG, 1, acc_gyr_on_values, 2, 10);
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDRESS, LSM6DSL_CTRL3_C, 1, reg_increment, 1, 10);
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDRESS, LSM6DSL_CTRL6_C, 1, acc_gyr_modes, 2, 10);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint16_t beginning = 0, end = 0, length = 0;
	while (1) {
		//==================================== GPS PPS ====================================
		if (gFlags.PPS) {
			// Atstatyti veliava
			gFlags.PPS = 0;
			TIM11->CNT = 0;

			// Isvalyti masyva
			memset(gGPS_UART_buffer, 0, sizeof gGPS_UART_buffer);

			// Nuskaityti GPS duomenis
			HAL_UART_Receive(&huart1, gGPS_UART_buffer, GPS_UART_BUFFER_SIZE, 150);

			// Resetint Timerio perioda ir duot zenkla kad reikia perziuret gps duomenis
			gFlags.GPSDataProcessed = 0;
		}

		//==================================== Timer IT ===================================
		if (gFlags.timer11) {
			gFlags.timer11 = 0;

			//________________ READING DATA FROM ACCELEROMETER AND GYROSCOPE
			// Turn on acc and reg, auto increment, set low/normal modes for acc and gyr, read data
			HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G, 1, gAccGyro.data_acc_gyr, 12, 10);

			//____________ UNPACKING DATA FROM ACCELEROMETER AND GYROSCOPE
//			for (uint8_t i = 0; i < 6; i++) {
//				int16_t temp = (gAccGyro.data_acc_gyr[2 * i + 1] << 8) | gAccGyro.data_acc_gyr[2 * i];
//				*((uint16_t*) &gAccGyro.acc_xx + i) = (int16_t) ((float) temp * (i < 3 ? 8.75 / 1000 : 0.061));
//			}

			if (!gFlags.GPSDataProcessed) {

				beginning = end = 0;
				for (uint16_t i = 1; i < strlen((char*) gGPS_UART_buffer); i++)
					if (beginning == 0) {
						if (gGPS_UART_buffer[i] == '$') beginning = i;
					} else if (gGPS_UART_buffer[i] == '\r') {
						end = i + 2;
						break;
					}
				length = end - beginning;

				pFullPeriodicPacket = (uint8_t*) realloc(pFullPeriodicPacket, length + 12);

				memcpy(pFullPeriodicPacket + 12, &gGPS_UART_buffer[beginning], length);
//				HAL_UART_Transmit(&huart6, pFullPeriodicPacket, length + 12, 10);
				gFlags.GPSDataProcessed = 1;
			}	// Naujausios eilutes is GPS masyvo paruosimas
//
//			// Atnaujinti sensoriu duomenis
			memcpy(pFullPeriodicPacket, &gAccGyro.acc_xx, 12);
			HAL_UART_Transmit(&huart6, pFullPeriodicPacket, length + 12, 10);
		}
//
//		//____________________READING DATA FROM TEMPERATURE AND HUMIDITY SENSOR
//
//		/*
//		 HAL_I2C_Master_Transmit(&hi2c1, writing_th, who_am_I_reg, 1, 10);
//		 HAL_Delay(30);
//		 HAL_I2C_Master_Receive(&hi2c1, reading_th, whoami, 1, 10);
//		 HAL_Delay(30);
//		 HAL_I2C_Master_Transmit(&hi2c1, writing_th, hum_temp_on, 2, 10);
//		 HAL_Delay(30);
//		 HAL_I2C_Master_Transmit(&hi2c1, writing_th, HUMIDITY_OUT_L, 1, 10);
//		 HAL_Delay(30);
//		 HAL_I2C_Master_Receive(&hi2c1, reading_th, data_temp_hum, 4, 10);
//		 HAL_Delay(03);
//		 //___________________UNPACKING DATA FROM TEMPERATURE AND HUMIDITY SENSOR
//		 k=0;
//		 for (int i = 0; i < 2; i++) { unpacked_temp_hum[i] = (data_temp_hum[k+1] << 8) | data_temp_hum[k]; k=k+2; }
//		 hum=unpacked_temp_hum[0];
//		 temp=unpacked_temp_hum[1];
//		 */
//
//		//___________________READING DATA FROM AKSELEROMETER AND MANGETOMETER
//		HAL_I2C_Mem_Write(&hi2c1, LSM303_WRITING_MAGN, LSM303_MAGN_ON_MODE, 1, magn_on, 1, 10);
//		HAL_I2C_Mem_Read(&hi2c1, LSM303_READING_MAGN, LSM303_OUTX_L_REG_M, 1, data_magn, 6, 10);
//
//		//___________________UNPACKING DATA, MAGNETOMETER
//		for (int i = 0, k = 0; i < 3; i++) {
//			unpacked_magn[i] = (data_magn[k + 1] << 8) | data_magn[k];
//			k = k + 2;
//		}
//		magn_x = unpacked_magn[0];
//		magn_y = unpacked_magn[1];
//		magn_z = unpacked_magn[2];
//
//		//___________________READING DATA FROM ACCELEROMETER
//		HAL_I2C_Mem_Write(&hi2c1, LSM303_WRITING_ACC2, LSM303_CTRL_REG1_A, 1, reg, 1, 10);
//		HAL_I2C_Mem_Read(&hi2c1, LSM303_READING_ACC2, LSM303_REG_OUT_ACC2, 1, data_acc2, 12, 10);
//
//		// twos complement left-justified xxxx xxxx xx00 0000
//		unpacked_acc2[0] = ((data_acc2[3] & 0xC0) << 18) | (data_acc2[2] << 10) | ((data_acc2[1] & 0xC0) << 8) | data_acc2[0];
//		unpacked_acc2[1] = ((data_acc2[7] & 0xC0) << 18) | (data_acc2[6] << 10) | ((data_acc2[5] & 0xC0) << 8) | data_acc2[4];
//		unpacked_acc2[2] = ((data_acc2[11] & 0xC0) << 18) | (data_acc2[10] << 10) | ((data_acc2[9] & 0xC0) << 8) | data_acc2[8];
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 64000 - 1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 245 - 1;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 230400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 921600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPS_Reset_GPIO_Port, GPS_Reset_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : GPS_WakeUp_Pin GPS_Reset_Pin */
	GPIO_InitStruct.Pin = GPS_WakeUp_Pin | GPS_Reset_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_10) {
		// Duoti zenkla, kad bus siunciami duomenys is GPS, juos nuskaityti while(0) cikle
		gFlags.PPS = 1;
	}
}

uint16_t getNewlineIndex(uint8_t *array, uint16_t size, uint16_t num) {
	if (num >= 1) {
		uint16_t counter = 0;
		for (uint16_t i = 0; i < size - 1; i++) {
			if (array[i] == 13 && array[i + 1] == 10) {
				counter++;

				if (counter == num) return i + 2;
			}
		}
	}
	return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);
	if (htim->Instance == TIM11) {
		gFlags.timer11 = 1;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
