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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "fdcan.h"
#include "hrtim.h"
#include "hrtim.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWR_FLAG_WUF PWR_FLAG_WUF2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint32_t Index = 1;
__IO uint32_t ErasingOnGoing = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void writeEEPROM8(uint32_t Index, uint8_t data);
void writeEEPROM16(uint32_t Index, uint16_t data);
void writeEEPROM32(uint32_t Index, uint32_t data);

uint8_t readEEPROM8(uint32_t Index);
uint16_t readEEPROM16(uint32_t Index);
uint32_t readEEPROM32(uint32_t Index);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//Set initial EE-Status
	EE_Status ee_status = EE_OK;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	LL_HRTIM_StartDLLCalibration(HRTIM1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//Enable FLASH Interrupt and set NVIC Priority
	HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FLASH_IRQn);

	//Unlock the Flash write protection
	HAL_FLASH_Unlock();

	//Enable RCC Clock
	__HAL_RCC_PWR_CLK_ENABLE();

	//Init the eeprom
	//If this fails, do an entire chip erase
	ee_status = EE_Init(EE_FORCED_ERASE);
	if (ee_status != EE_OK)
	{
		Error_Handler();
	}
	//Set initial sample rate
	targetSampleDelay = 10000;
	//Set initial Target Voltage
	targetVout = 5000;
	//Set initial conversion mode
	conversionState = CONVERSION_STATE_SHUTDOWN;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_UART5_Init();
  MX_USB_Device_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	LL_ADC_DisableDeepPowerDown(ADC1);
	HAL_Delay(1);	//Wait before powering up the regulator
	LL_ADC_EnableInternalRegulator(ADC1);
	HAL_Delay(1);	//Wait for regulator stabilization
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
	HAL_Delay(1);	//Wait after calibration before activation ADC
	LL_ADC_Enable(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
	HAL_Delay(1);	//Wait before starting conversion
	//Start Timer and ADC Peripherals
	LL_ADC_INJ_StartConversion(ADC1);
	LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_E | LL_HRTIM_TIMER_F);
	//Init ADC Filters
	Moving_Average_Init(&VinFilter);
	Moving_Average_Init(&VoutFilter);
	Moving_Average_Init(&CurInFilter);
	Moving_Average_Init(&CurOutFilter);
	//Enable Repetition Interrupt HRTIM
	LL_HRTIM_EnableIT_REP(HRTIM1, LL_HRTIM_TIMER_F);
	//Enable End of Injected Conversion Interrupt ADC
	LL_ADC_EnableIT_JEOS(ADC1);
	//Enable 32Bit Timer for timing us events
	HAL_TIM_Base_Start(&htim5);
	__HAL_TIM_SET_PRESCALER(&htim5, HAL_RCC_GetPCLK1Freq()/1000000 - 1);
	HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t nextSampleTime = 0;

	while (1)
	{
		if(__HAL_TIM_GET_COUNTER(&htim5) >= nextSampleTime)
		{
			hostInterfaceQueueDeviceCMDExpl(OPSTATUS_OK, OPCODE_REPORT_LIVE_VALUE, OPTYPE_OUTPUT_VOLTAGE, VoutAverage);
			hostInterfaceQueueDeviceCMDExpl(OPSTATUS_OK, OPCODE_REPORT_LIVE_VALUE, OPTYPE_INPUT_VOLTAGE, VinAverage);
			hostInterfaceQueueDeviceCMDExpl(OPSTATUS_OK, OPCODE_FINISHED_SAMPLE, OPTYPE_SAMPLE_NUM_OF_CHANNELS, 2);

			nextSampleTime = __HAL_TIM_GET_COUNTER(&htim5) + targetSampleDelay;
		}

		hostInterfaceProcessCommand();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_SYSCLK;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
int _write(int32_t file, uint8_t *ptr, int32_t len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for(i=0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}
/**
 * @brief  FLASH end of operation interrupt callback.
 * @param  ReturnValue: The value saved in this parameter depends on the ongoing procedure
 *                  Mass Erase: Bank number which has been requested to erase
 *                  Page Erase: Page which has been erased
 *                    (if 0xFFFFFFFF, it means that all the selected pages have been erased)
 *                  Program: Address which was selected for data program
 * @retval None
 */
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
	/* Call CleanUp callback when all requested pages have been erased */
	if (ReturnValue == 0xFFFFFFFF)
	{
		EE_EndOfCleanup_UserCallback();
	}
}

/**
 * @brief  Clean Up end of operation interrupt callback.
 * @param  None
 * @retval None
 */
void EE_EndOfCleanup_UserCallback(void)
{
	ErasingOnGoing = 0;
}

void writeEEPROM8(uint32_t Index, uint8_t data)
{
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1)
	{
	}

	EE_Status ee_status = EE_WriteVariable8bits(Index, data);

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP)
	{
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR)
	{
		Error_Handler();
	}
}

void writeEEPROM16(uint32_t Index, uint16_t data)
{
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1)
	{
	}

	EE_Status ee_status = EE_WriteVariable16bits(Index, data);

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP)
	{
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR)
	{
		Error_Handler();
	}
}

void writeEEPROM32(uint32_t Index, uint32_t data)
{
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1)
	{
	}

	EE_Status ee_status = EE_WriteVariable32bits(Index, data);

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP)
	{
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR)
	{
		Error_Handler();
	}
}

uint8_t readEEPROM8(uint32_t Index)
{
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1)
	{
	}

	uint8_t dataOut = 0;

	EE_Status ee_status = EE_ReadVariable8bits(Index, &dataOut);

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP)
	{
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR)
	{
		Error_Handler();
	}
	return dataOut;
}

uint16_t readEEPROM16(uint32_t Index)
{
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1)
	{
	}

	uint16_t dataOut = 0;

	EE_Status ee_status = EE_ReadVariable16bits(Index, &dataOut);

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP)
	{
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR)
	{
		Error_Handler();
	}
	return dataOut;
}

uint32_t readEEPROM32(uint32_t Index)
{
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1)
	{
	}

	uint32_t dataOut = 0;

	EE_Status ee_status = EE_ReadVariable32bits(Index, &dataOut);

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP)
	{
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR)
	{
		Error_Handler();
	}
	return dataOut;
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
	__disable_irq();
	while (1)
	{
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
