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

#include "stm32g4xx_ll_crc.h"
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
#include "stm32g4xx_ll_hrtim.h"
#include "moving_average.h"
#include "eeprom_emul.h"
#include <hostInterface.h>
#include "spi.h"
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
#define CHGPMP_PERIOD 34000
#define PWM_PERIOD 27200
#define DBG_LED4_Pin GPIO_PIN_13
#define DBG_LED4_GPIO_Port GPIOC
#define DBG_LED3_Pin GPIO_PIN_14
#define DBG_LED3_GPIO_Port GPIOC
#define DBG_LED2_Pin GPIO_PIN_15
#define DBG_LED2_GPIO_Port GPIOC
#define DBG_LED1_Pin GPIO_PIN_2
#define DBG_LED1_GPIO_Port GPIOC
#define VIN_CHGPMP_Pin GPIO_PIN_3
#define VIN_CHGPMP_GPIO_Port GPIOC
#define VOUT_CHGPMP_Pin GPIO_PIN_2
#define VOUT_CHGPMP_GPIO_Port GPIOB
#define BQ_NSS_Pin GPIO_PIN_12
#define BQ_NSS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef float float32_t;

volatile unsigned int bms_present;

#define MAX_VCELL   4.2f
#define MIN_VCELL   2.5f

#define UPPER_DC_LIMIT_BUCK	PWM_PERIOD * 0.95
#define LOWER_DC_LIMIT_BUCK	PWM_PERIOD * 0.05
#define UPPER_DC_LIMIT_BOOST	PWM_PERIOD * 0.95
#define LOWER_DC_LIMIT_BOOST	PWM_PERIOD * 0.05
#define UPPER_DC_LIMIT_BUCKBOOST	PWM_PERIOD * 0.95
#define LOWER_DC_LIMIT_BUCKBOOST	PWM_PERIOD * 0.05
//Conversion States
#define CONVERSION_STATE_SHUTDOWN	0
#define CONVERSION_STATE_BUCK		1
#define CONVERSION_STATE_BOOST		2
#define CONVERSION_STATE_BUCKBOOST	3
//Shutdown Power Conversion if any of the inputs/outputs exceed this limit [mV]
#define OVER_VOLTAGE_PROTECTION		30000
//BUCK-BOOST Conversion Mode Band [mV]
#define BUCK_BOOST_BAND 								2000
#define BUCK_BOOST_BAND_HYSTERESIS 			400
//Analog Supply Voltage [mV]
#define VDDA						3300
//Oversampling correction factor
#define ADC_OVERSAMPLING_X2		0x1FFE
#define ADC_OVERSAMPLING_X4		0x3FFC
#define ADC_OVERSAMPLING_X8		0x7FF8
#define ADC_OVERSAMPLING_X16	0xFFF0

//Input Voltage [counts] [mV] [mV]
volatile unsigned int vinRawADC;
volatile unsigned int vinRawVolt;
volatile unsigned int Vin;
volatile unsigned int VinAverage;
FilterTypeDef VinFilter;
#define VIN_VDIV_RUP	820
#define VIN_VDIV_RLOW	43

//Input Current [counts] [mA] [mA]
volatile unsigned int curInRawADC;
volatile int curInRawVolt;
volatile int CurIn;
volatile int CurInAverage;
FilterTypeDef CurInFilter;
//#define CURIN_AMP_FACTOR 20			//INA240A1 Amplification
#define CURIN_AMP_FACTOR 		50		//INA240A2 Amplification
#define CURIN_SHUNT_FAC			200		//1 / ShuntResistance = 1 / 5mR = 200
#define CURIN_VOLTAGE_REF 	340		//340mV Shunt Amplifier Offset


//Output Voltage [counts] [mV] [mV]
volatile unsigned int voutRawADC;
volatile unsigned int voutRawVolt;
volatile unsigned int Vout;
volatile unsigned int VoutAverage;
FilterTypeDef VoutFilter;
#define VOUT_VDIV_RUP		820
#define VOUT_VDIV_RLOW	43

//Output Current [counts] [mA] [mA]
volatile unsigned int curOutRawADC;
volatile int curOutRawVolt;
volatile int CurOut;
volatile int CurOutAverage;
FilterTypeDef CurOutFilter;
//#define CURIN_AMP_FACTOR 	20		//INA240A1 Amplification
#define CUROUT_AMP_FACTOR 	50		//INA240A2 Amplification
#define CUROUT_SHUNT_FAC		200		//1 / ShuntResistance = 1 / 5mR = 200
#define CUROUT_VOLTAGE_REF 	340		//340mV Shunt Amplifier Offset


//Target Output Voltage [mV] (CV)
volatile unsigned int targetVout;
//Target Output Voltage [mA] (CC)
volatile unsigned int limitIout;
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
