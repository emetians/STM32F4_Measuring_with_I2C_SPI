/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// BMP180 Parameter Addresses
#define BMP180ADR_CALIB_START_ADR 0xAA
#define BMP180ADR_CALIB_ADR_COUNT 22
#define BMP180ADR_CTRL_REG        0XF4

#define BMP180_READ_ADR           0xEF
#define BMP180_WRITE_ADR          0xEE
#define BMP180_TMP_ADR            0x2E
#define BMP180_PRS_ADR 	 		  0xB4 // for when oss=0b01
#define BMP180_OUT_XLSB_ADR 	  0xF8
#define BMP180_OUT_LSB_ADR 		  0xF7
#define BMP180_OUT_MSB_ADR 		  0xF6

//MPU9250 Addresses

#define MPU9250_SLV0_ADDR 		  0x25
#define MPU9250_SLV0_REG 		  0x26
#define MPU9250_SLV0_CTRL 		  0x27
#define MPU9250_USER_CTRL		  0x6A
#define MPU9250_PWR_MGMT_1        0x6B
#define MPU9250_PWR_MGMT_2		  0x6C
#define MPU9250_WHOAMI			  0x75
#define MPU9250_ACCEL_CONFIG	  0x1C
#define MPU9250_ACCEL_CONFIG_2	  0x1D
#define MPU9250_SMPLRT_DIV		  0x19
#define MPU9250_ACCEL_XOUT_H      0x3B
#define MPU9250_ACCEL_XOUT_L      0x3C
#define MPU9250_ACCEL_YOUT_H      0x3D
#define MPU9250_ACCEL_YOUT_L      0x3E
#define MPU9250_ACCEL_ZOUT_H      0x3F
#define MPU9250_ACCEL_ZOUT_L      0x40




// Calibration parameters
struct bmp180_calib_params{
	short int ac1; 			//calibration ac1 data
    short int ac2; 			//calibration ac2 data
    short int ac3; 			//calibration ac3 data
    unsigned short int ac4; //calibration ac4 data
    unsigned short int ac5; //calibration ac5 data
    unsigned short int ac6; //calibration ac6 data
    short int b1; 			//calibration b1 data
    short int b2; 			//calibration b2 data
    short int mb; 			//calibration mb data
    short int mc; 			//calibration mc data
    short int md; 			//calibration md data


}bmp_params;








/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
