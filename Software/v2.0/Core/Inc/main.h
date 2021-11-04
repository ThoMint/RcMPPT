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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_crc.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "moving_average.h"
#include "eeprom_emul.h"
#include <hostInterface.h>
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
#define PWM_PERIOD 21760
#define TP1_Pin GPIO_PIN_4
#define TP1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define UPPER_DC_LIMIT_BUCK	PWM_PERIOD * 0.95
#define LOWER_DC_LIMIT_BUCK	PWM_PERIOD * 0.05
#define UPPER_DC_LIMIT_BOOST	PWM_PERIOD * 0.70
#define LOWER_DC_LIMIT_BOOST	PWM_PERIOD * 0.05
#define UPPER_DC_LIMIT_BUCKBOOST	PWM_PERIOD * 0.70
#define LOWER_DC_LIMIT_BUCKBOOST	PWM_PERIOD * 0.05
//Conversion States
#define CONVERSION_STATE_SHUTDOWN	0
#define CONVERSION_STATE_BUCK		1
#define CONVERSION_STATE_BOOST		2
#define CONVERSION_STATE_BUCKBOOST	3
//Shutdown Power Conversion if any of the inputs/outputs exceed this limit [mV]
#define OVER_VOLTAGE_PROTECTION		20000
//BUCK-BOOST Conversion Mode Band [mV]
#define BUCK_BOOST_BAND 								1200
#define BUCK_BOOST_BAND_HYSTERESIS 			200
//Analog Supply Voltage [mV]
#define VDDA						3300
//Input Voltage [counts] [mV] [mV]
volatile unsigned int vinRawADC;
volatile unsigned int vinRawVolt;
volatile unsigned int Vin;
volatile unsigned int VinAverage;
FilterTypeDef VinFilter;
#define VIN_CORRECTION_FACTOR	(19.0/18.4)

//Input Current [counts] [mA] [mA]
volatile unsigned int curInRawADC;
volatile unsigned int curInRawVolt;
volatile unsigned int CurIn;
volatile unsigned int CurInAverage;
FilterTypeDef CurInFilter;
#define CURIN_CORRECTION_FACTOR	(1.566/1.810)

//Output Voltage [counts] [mV] [mV]
volatile unsigned int voutRawADC;
volatile unsigned int voutRawVolt;
volatile unsigned int Vout;
volatile unsigned int VoutAverage;
FilterTypeDef VoutFilter;
#define VOUT_CORRECTION_FACTOR	(17.4/17.0)

//Output Current [counts] [mA] [mA]
volatile unsigned int curOutRawADC;
volatile unsigned int curOutRawVolt;
volatile unsigned int CurOut;
volatile unsigned int CurOutAverage;
FilterTypeDef CurOutFilter;
#define CUROUT_CORRECTION_FACTOR	(1.63/2.0)

//Target Output Voltage [mV] (CV)
volatile unsigned int targetVout;
//Target Output Voltage [mA] (CC)
volatile unsigned int targetIout;
//Target Sample Delay [us] (1/SR)
volatile unsigned int targetSampleDelay;
//BuckBoost Band [mV] (adaptive)
volatile unsigned int buckBoostBand;

//Conversion State: Buck, Boost, Buck-Boost
volatile uint8_t conversionState;
volatile uint8_t prevConversionState;
volatile unsigned int CurrentDuty;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
