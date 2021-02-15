/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define max_strlen 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Usart_Menu(char *input);
void BMP180_Init();
float BMP180_Read_Pressure();
float BMP180_Read_Temperature();
void MPU9250_Init();
float * MPU9250_Read_Accels();

char msg[max_strlen + 1] = "Merhaba\n";
char rxbuffer;
char received_string[max_strlen + 1];
char usart_data[max_strlen + 1];
uint8_t cnt = 0;
uint8_t i = 0;
uint8_t i2cbuff[12];
HAL_StatusTypeDef state;
uint8_t ctrl = 0b01000000; // for standart mode(number of samples=2 conversion time= 7.5ms) used 6-7/ it means oss=0b01
uint8_t oss = 1;
long B5;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//byte to word   word=(msb<<8)+lsb
void BMP180_Init() {
	HAL_StatusTypeDef state;



	state = HAL_I2C_Mem_Write(&hi2c1, BMP180_WRITE_ADR, BMP180ADR_CTRL_REG, 1, &ctrl, 1, 10);  //1. Write oversampling settings to control register
	HAL_Delay(7.5);
	uint8_t data[BMP180ADR_CALIB_ADR_COUNT];

	if (state == HAL_OK) {

		HAL_I2C_Mem_Read(&hi2c1, BMP180_READ_ADR, BMP180ADR_CALIB_START_ADR, 1,data, BMP180ADR_CALIB_ADR_COUNT, 10); // 2. Read Calibration Data from device

		bmp_params.ac1 = (data[0] << 8) + data[1];
		bmp_params.ac2 = (data[2] << 8) + data[3];
		bmp_params.ac3 = (data[4] << 8) + data[5];
		bmp_params.ac4 = (data[6] << 8) + data[7];
		bmp_params.ac5 = (data[8] << 8) + data[8];
		bmp_params.ac6 = (data[10] << 8) + data[11];
		bmp_params.b1  = (data[12] << 8) + data[13];
		bmp_params.b2  = (data[14] << 8) + data[15];
		bmp_params.mb  = (data[16] << 8) + data[17];
		bmp_params.mc  = (data[18] << 8) + data[19];
		bmp_params.md  = (data[20] << 8) + data[11];




	}else{
		sprintf(msg, "BMP180 Error!\n");
		HAL_UART_Transmit_IT(&huart2, (uint8_t *) msg, strlen(msg));

		while(1){
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			HAL_Delay(100);
		}
	}

}

float BMP180_Read_Pressure() {

	uint8_t buf[3] = {0};
	HAL_I2C_Mem_Write(&hi2c1, BMP180_WRITE_ADR, BMP180ADR_CTRL_REG, 1, (uint8_t *)BMP180_PRS_ADR, 1, 10); // if we put pressure address to control reg we can read pressure value from out registers
	HAL_Delay(8);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_READ_ADR, BMP180_OUT_MSB_ADR, 1, &buf[0], 1, 10);                           // reading pressure values from device
	HAL_I2C_Mem_Read(&hi2c1, BMP180_READ_ADR, BMP180_OUT_LSB_ADR, 1, &buf[1], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_READ_ADR, BMP180_OUT_XLSB_ADR, 1, &buf[2], 1, 10);

	//Calculate pressure value
	long pressure;
	long UP = (buf[0] << 16) + (buf[1] << 8) + (buf[2] >> (8 - oss));
	long B6 = B5 - 4000;
	long X1 = ((bmp_params.b2 * ((B6 * B6) / pow(2, 12)))) / pow(2, 11);
	long X2 = (bmp_params.ac2 * B6) / pow(2, 11);
	long X3 = X1 + X2;
	long B3 = (((bmp_params.ac1 * 4 + X3) << oss) + 2) / 4;
	X1 = (bmp_params.ac3 * B6) / pow(2, 13);
	X2 = (bmp_params.b1 * ((B6 * B6) / pow(2, 12))) / pow(2, 16);
	X3 = ((X1 + X2) + 2) / (2 * 2);
	long B4 = bmp_params.ac4 * (unsigned long)(X3 + 32768) / pow(2, 15);
	long B7 = ((unsigned long)UP - B3) * (50000 >> oss);

	if(B7 < 0x80000000){pressure = (B7 * 2) / B4;}
	else{pressure = (B7 / B4) * 2;}

	X1 = (pressure / pow(2, 8)) * (pressure / pow(2, 8));
	X1 = (X1 * 3038) / pow(2, 16);
	X2 = (-7357 * pressure) / pow(2, 16);
	pressure = pressure + (X1 + X2 + 3791) / pow(2, 4);


	return (float)pressure;
}


float BMP180_Read_Temperature(){

	uint8_t buf[2] = {0};
	HAL_I2C_Mem_Write(&hi2c1, BMP180_WRITE_ADR, BMP180ADR_CTRL_REG, 1, (uint8_t *)BMP180_TMP_ADR, 1, 10); // if we put temp. address to control reg we can read temp value from out registers
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_READ_ADR, BMP180_OUT_LSB_ADR, 1, &buf[0], 1, 10);					   	// getting temp. values from device
	HAL_I2C_Mem_Read(&hi2c1, BMP180_READ_ADR, BMP180_OUT_MSB_ADR, 1, &buf[1], 1, 10);

	long UT = (buf[1] << 8) + buf[0];      // Uncompansated temp value

	//Calculate temp value
	long X1 = (UT - bmp_params.ac6) * (bmp_params.ac5 /pow(2, 15));
	long X2 = (bmp_params.mc * pow(2, 11)) / (X1 + bmp_params.md);
	B5 = X1 + X2;
	long temp = (B5 + 8) / pow(2, 4);


	return (float)temp;
}

void MPU9250_Init(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // NCS Pin High to enable chip

	HAL_StatusTypeDef state;


	uint8_t rx;

	state = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)MPU9250_WHOAMI, &rx, 1, 50);  //Check WHO AM I register in MPU9250. It should be equal to reset value(0x71) by datasheet.


	if(rx != 0x71 || state != HAL_OK)
		Error_Handler();

	HAL_SPI_Transmit(&hspi1, (uint8_t *)MPU9250_PWR_MGMT_1, 1, 50); // Send 0x80 to pwr_mgmt_1 register for reset chip
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0x08, 1, 50);
	HAL_Delay(10); // wait for chip to back


	HAL_SPI_Transmit(&hspi1, (uint8_t *)MPU9250_PWR_MGMT_2, 1, 50); // Send 0x00 to pwr_mgmt_2 register for enable accelerometer and gyro
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0x00, 1, 50);

	HAL_SPI_Transmit(&hspi1, (uint8_t *)MPU9250_ACCEL_CONFIG, 1, 50); // Send 0x18 to accel_config register for set accelerometer range 16g
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0x18, 1, 50);

	HAL_SPI_Transmit(&hspi1, (uint8_t *)MPU9250_ACCEL_CONFIG_2, 1, 50); // Send 0x01 to accel_config_2 register for set bandwith 184Hz
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0x01, 1, 50);


	HAL_SPI_Transmit(&hspi1, (uint8_t *)MPU9250_SMPLRT_DIV, 1, 50); // Send 0x00 to smplrt_div register for update rate divider as 0
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0x00, 1, 50);

	HAL_SPI_Transmit(&hspi1, (uint8_t *)MPU9250_USER_CTRL, 1, 50); // Send 0x10 to user_ctrl register for disable i2c and enable spi mode
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0x10, 1, 50);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // NCS Pin low to disable chip

	//MPU9250 communication began.

}

float * MPU9250_Read_Accels(){  // This function is returning pointer to acces 3-axis accel data from device with only 1 function calling
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // NCS Pin High to enable chip

	uint8_t accels[6];
	static short * results[3];


	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)MPU9250_ACCEL_XOUT_H, (uint8_t *)accels, 6);  // start reading from XOUT_H register to ZOUT_L sequential

	*results[0] = (accels[0]<<8) + accels[1]; // acceleration value of x-axis
	*results[1] = (accels[2]<<8) + accels[3]; // acceleration value of y-axis
	*results[2] = (accels[4]<<8) + accels[5]; // acceleration value of z-axis

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // NCS Pin low to disable chip

	return (float *)results;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {  // This Function work when receive data from usart
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	/* NOTE: This function should not be modified, when the callback is needed,
	 the HAL_UART_RxCpltCallback could be implemented in the user file
	 */



	if ((rxbuffer != '\n') && (cnt < max_strlen)) { // Read usart data as 1 char then combine them all as char array
		                                            // until receive '\n' character or reach to max string length

		received_string[cnt] = rxbuffer;

		cnt++;
		i = cnt;

	} else {										// When complete the receiving data then copy to usart_data variable and remove received_string for new data receive

		cnt = 0;
		strcpy(usart_data, received_string);

		Usart_Menu(usart_data);						// send usart_data to Usart_Menu function for general operations

		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		memset(&received_string, 0, sizeof(received_string)); // remove received_string

	}

}

void Usart_Menu(char *input) {

	float *a;                    /* pointer for hold accelerometer values
	 	 	 	 	 	 	 	 	a -> x-axis
	 	 	 	 	 	 	 	 	a + 1 -> y axis
	 	 	 	 	 	 	 	 	a + 2 -> z axis*/


	a = MPU9250_Read_Accels();


	float ay = *(a + 1);
	float az = *(a + 2);
	float angle = atan2(ay, az)*180/M_PI;   /*Calculate roll angle
	 	 	 	 	 	 	 	 	 	 	  !!!atan2 function(arc tangent) returns value as rad/s
	 	 	 	 	 	 	 	 	 	 	  we can calculate degree with multiply 180/M_PI*/

	if (!strcmp(input, "p")) {										   // if receive 'p' from usart send pressure value back
		sprintf(msg, "Pressure: %f\n", BMP180_Read_Temperature());
		HAL_UART_Transmit_IT(&huart2, (uint8_t *) msg, strlen(msg));
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

	} else if (!strcmp(input, "t")) {
		sprintf(msg, "Temperature: %f\n", BMP180_Read_Temperature()); // if receive 't' from usart send temperature value back
		HAL_UART_Transmit_IT(&huart2, (uint8_t *) msg, strlen(msg));

	} else if (!strcmp(input, "a")) {
		sprintf(msg, "Angle: %f\n",angle);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *) msg, strlen(msg));  // if receive 'a' from usart send angle value back

	} else if (!strcmp(input, "all")) {								  // if receive 'all' from usart send all value back
		sprintf(msg, "Pressure: %f Temp: %f Angle: %f\n", BMP180_Read_Pressure(), BMP180_Read_Temperature(), angle);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *) msg, strlen(msg));

	}

}

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  HAL_UART_Transmit_IT(&huart2, (uint8_t *) msg, strlen(msg));

  __HAL_SPI_ENABLE(&hspi1);
  BMP180_Init();
  MPU9250_Init();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		HAL_Delay(10);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 82;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
