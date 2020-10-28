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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

struct {
	uint8_t PPS:1;
	uint8_t unused:7;
} mFlags;

uint8_t mGPS_UART_Buffer[1000];

#define acc_gyr_address 0xD7
#define CTRL3_C 0x12 // control register for incrementing
#define acc_on_reg 0x10 // acc mode (turns on acc)
#define CTRL6_C 0x15 // control register for acc modes
#define OUTX_L_G 0x22 // first register of gyr reading
uint8_t reg_increment[1] = {0x04}; // register increment
uint8_t acc_gyr_on_values[2] = {0x50, 0x50}; // turn on acc and gyr
uint8_t acc_gyr_modes[2] = {0x10,0x80}; // low/normal modes for acc and gyr
uint8_t data_acc_gyr[12]; // data received from acc and gyr sensors
int16_t unpacked[6], acc_xx, acc_yy, acc_zz, gyr_xx, gyr_yy, gyr_zz;

//temperature and humidity variables
/*
#define temp_hum_address 0xBC
#define writing_th 0xBE
#define reading_th 0xBF
uint8_t hum_temp_on[2] ={0x20, 0x83}; //turn on sensor + 12.5Hz, NOT one-shot
uint8_t HUMIDITY_OUT_L[1] = {0x28};
uint8_t data_temp_hum[4];
uint8_t who_am_I_reg[1] = {0x0F};
uint8_t whoami[1];
int16_t unpacked_temp_hum[2], temp, hum;
*/
//magnetometer variables
#define magn_address 0x33 // magnetometer sensor adress
#define reading_magn 0x3D // adress to read from magn sensor registers
#define writing_magn 0x3C // address to write to magn sensor registers
#define magn_on_mode 0x60 // control register to turn on magnetometer
#define OUTX_L_REG_M 0x68|0x80 // magnetometer XX YY ZZ
uint8_t magn_on[1] = {0x1C}; // continuous/single measurement mode
uint8_t data_magn[6];
int16_t unpacked_magn[3], magn_x, magn_y, magn_z;

//ACC
#define writing_acc2 0x32 // address to write to acc registers
#define reading_acc2 0x33 // address to read from acc registers
#define CTRL_REG1_A 0x20 // modes register
#define reg_out_acc2 0x28 // first register to output data
uint8_t reg[1] = {0x57}; // normal mode on
uint8_t data_acc2[12];
int32_t unpacked_acc2[3];

uint16_t checkout;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

uint16_t getNewlineIndex(uint8_t* array, uint16_t size, uint16_t num);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t beginning=0, end=0, ilgis;
  while (1)
  {
	  if(mFlags.PPS){
		  // Atstatyti veliava
		  mFlags.PPS=0;

		  // Nuskaityti GPS duomenis
		  HAL_UART_Receive(&huart1, mGPS_UART_Buffer, 1000, 800);

		  // Surasti nuskaitytos zinutes ilgi
		  ilgis=strlen((char*)mGPS_UART_Buffer);
		  HAL_UART_Transmit_DMA(&huart6, mGPS_UART_Buffer, ilgis);	// Pilnas paketas

		  // Surasti reikiamos eilutes pradzia ir pabaiga
		  beginning = getNewlineIndex(mGPS_UART_Buffer, ilgis, 1);
		  end = getNewlineIndex(mGPS_UART_Buffer, ilgis, 2);

		  ilgis=end-beginning;	// Surinktos eilutes ilgis

		  // Persiusti masyva per UART
		  HAL_UART_Transmit_DMA(&huart6, &mGPS_UART_Buffer[beginning], ilgis);

		  // Nunulinti masyva
		  memset(mGPS_UART_Buffer, 0, sizeof mGPS_UART_Buffer);
	  }

	  //________________ READING DATA FROM ACCELEROMETER AND GYROSCOPE
	  // Turn on acc and reg, auto increment, set low/normal modes for acc and gyr, read data
		  HAL_I2C_Mem_Write(&hi2c1, acc_gyr_address, acc_on_reg, 1, acc_gyr_on_values, 2, 10);
		  HAL_I2C_Mem_Write(&hi2c1, acc_gyr_address, CTRL3_C, 1, reg_increment, 1, 10);
		  HAL_I2C_Mem_Write(&hi2c1, acc_gyr_address, CTRL6_C, 1, acc_gyr_modes, 2, 10);
		  HAL_I2C_Mem_Read(&hi2c1, acc_gyr_address, OUTX_L_G, 1, data_acc_gyr, 12, 10);
		  //____________ UNPACKING DATA FROM ACCELEROMETER AND GYROSCOPE
		  int k = 0;
		  for (int i = 0; i < 6; i++) { unpacked[i] = (data_acc_gyr[k+1] << 8) | data_acc_gyr[k]; k=k+2; }
		  gyr_xx=unpacked[0]*8.75/1000;
		  gyr_yy=unpacked[1]*8.75/1000;
		  gyr_zz=unpacked[2]*8.75/1000;
		  acc_xx=unpacked[3]*0.061;
		  acc_yy=unpacked[4]*0.061;
		  acc_zz=unpacked[5]*0.061;

		  //____________________READING DATA FROM TEMPERATURE AND HUMIDITY SENSOR

		  /*
		  HAL_I2C_Master_Transmit(&hi2c1, writing_th, who_am_I_reg, 1, 10);
		  HAL_Delay(30);
		  HAL_I2C_Master_Receive(&hi2c1, reading_th, whoami, 1, 10);
		  HAL_Delay(30);
		  HAL_I2C_Master_Transmit(&hi2c1, writing_th, hum_temp_on, 2, 10);
		  HAL_Delay(30);
		  HAL_I2C_Master_Transmit(&hi2c1, writing_th, HUMIDITY_OUT_L, 1, 10);
		  HAL_Delay(30);
		  HAL_I2C_Master_Receive(&hi2c1, reading_th, data_temp_hum, 4, 10);
		  HAL_Delay(03);
		  //___________________UNPACKING DATA FROM TEMPERATURE AND HUMIDITY SENSOR
		  k=0;
		  for (int i = 0; i < 2; i++) { unpacked_temp_hum[i] = (data_temp_hum[k+1] << 8) | data_temp_hum[k]; k=k+2; }
		  hum=unpacked_temp_hum[0];
		  temp=unpacked_temp_hum[1];
		  */

		  //___________________READING DATA FROM AKSELEROMETER AND MANGETOMETER

		  HAL_I2C_Mem_Write(&hi2c1, writing_magn, magn_on_mode, 1, magn_on, 1, 10);
		  HAL_I2C_Mem_Read(&hi2c1, reading_magn, OUTX_L_REG_M, 1, data_magn, 6, 10);

		  //___________________UNPACKING DATA, MAGNETOMETER
		  k=0;
		  for (int i = 0; i < 3; i++) { unpacked_magn[i] = (data_magn[k+1] << 8) | data_magn[k]; k=k+2; }
		  magn_x=unpacked_magn[0];
		  magn_y=unpacked_magn[1];
		  magn_z=unpacked_magn[2];

		  //___________________READING DATA FROM ACCELEROMETER
		  HAL_I2C_Mem_Write(&hi2c1, writing_acc2, CTRL_REG1_A, 1, reg, 1, 10);
		  HAL_I2C_Mem_Read(&hi2c1, reading_acc2, reg_out_acc2, 1, data_acc2, 12, 10);

		  // twos complement left-justified xxxx xxxx xx00 0000
		  unpacked_acc2[0] = ((data_acc2[3] & 0xC0) << 18) | (data_acc2[2] << 10 ) | ((data_acc2[1] & 0xC0) << 8) | data_acc2[0];
		  unpacked_acc2[1] = ((data_acc2[7] & 0xC0) << 18) | (data_acc2[6] << 10 ) | ((data_acc2[5] & 0xC0) << 8) | data_acc2[4];
		  unpacked_acc2[2] = ((data_acc2[11] & 0xC0) << 18) | (data_acc2[10] << 10 ) | ((data_acc2[9] & 0xC0) << 8) | data_acc2[8];
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 230400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_Reset_GPIO_Port, GPS_Reset_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : GPS_WakeUp_Pin GPS_Reset_Pin */
  GPIO_InitStruct.Pin = GPS_WakeUp_Pin|GPS_Reset_Pin;
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_10){
	  // Duoti zenkla, kad bus siunciami duomenys is GPS, juos nuskaityti while(0) cikle
	  mFlags.PPS = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  if(huart==&huart6){
	  HAL_UART_Transmit(&huart6, mGPS_UART_Buffer, 700, 10);
  }
}

uint16_t getNewlineIndex(uint8_t* array, uint16_t size, uint16_t num){
	if(num>=1){
		uint16_t counter=0;
		for(uint16_t i=0;i<size-1;i++){
			if(array[i]==13 && array[i+1]==10){
				counter++;

				if(counter==num)
					return i+2;
			}
		}
	}
	return 0;
}
/* USER CODE END 4 */

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
