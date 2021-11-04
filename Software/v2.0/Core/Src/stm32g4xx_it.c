/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* During the cleanup phase in EE_Init, AddressRead is the address being read */
extern __IO uint32_t AddressRead;
/* Flag equal to 1 when the cleanup phase is in progress, 0 if not */
extern __IO uint8_t CleanupPhase;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
inline uint32_t constrain(uint32_t input, uint32_t uppperLimit, uint32_t lowerLimit);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */
	/* Check if NMI is due to flash ECCD (error detection) */
	if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD))
	{
		if (CleanupPhase == 1)
		{
			if ((AddressRead >= START_PAGE_ADDRESS) && (AddressRead <= END_EEPROM_ADDRESS))
			{
				/* Delete the corrupted flash address */
				if (EE_DeleteCorruptedFlashAddress((uint32_t) AddressRead) == EE_OK)
				{
					/* Resume execution if deletion succeeds */
					return;
				}
				/* If we do not succeed to delete the corrupted flash address */
				/* This might be because we try to write 0 at a line already considered at 0 which is a forbidden operation */
				/* This problem triggers PROGERR, PGAERR and PGSERR flags */
				else
				{
					/* We check if the flags concerned have been triggered */
					if ((__HAL_FLASH_GET_FLAG(FLASH_FLAG_PROGERR)) && (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR)) && (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR)))
					{
						/* If yes, we clear them */
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PROGERR);
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);

						/* And we exit from NMI without doing anything */
						/* We do not invalidate that line because it is not programmable at 0 till the next page erase */
						/* The only consequence is that this line will trigger a new NMI later */
						return;
					}
				}
			}
		}
		else
		{
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
			return;
		}
	}
	/* USER CODE END NonMaskableInt_IRQn 0 */
	HAL_RCC_NMI_IRQHandler();
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1)
	{

	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */
	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles PVD/PVM1/PVM2/PVM3/PVM4 interrupts through EXTI lines 16/38/39/40/41.
 */
void PVD_PVM_IRQHandler(void)
{
	/* USER CODE BEGIN PVD_PVM_IRQn 0 */
	while (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET)
	{

	}
	/* USER CODE END PVD_PVM_IRQn 0 */
	HAL_PWREx_PVD_PVM_IRQHandler();
	/* USER CODE BEGIN PVD_PVM_IRQn 1 */

	/* USER CODE END PVD_PVM_IRQn 1 */
}

/**
 * @brief This function handles Flash global interrupt.
 */
void FLASH_IRQHandler(void)
{
	/* USER CODE BEGIN FLASH_IRQn 0 */

	/* USER CODE END FLASH_IRQn 0 */
	HAL_FLASH_IRQHandler();
	/* USER CODE BEGIN FLASH_IRQn 1 */

	/* USER CODE END FLASH_IRQn 1 */
}

/**
 * @brief This function handles ADC1 and ADC2 global interrupt.
 */
void ADC1_2_IRQHandler(void)
{
	/* USER CODE BEGIN ADC1_2_IRQn 0 */
	LL_ADC_ClearFlag_JEOS(ADC1);
	/* USER CODE END ADC1_2_IRQn 0 */

	/* USER CODE BEGIN ADC1_2_IRQn 1 */
	vinRawADC = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_1);

	voutRawADC = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_2);

	curInRawADC = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_3);

	curOutRawADC = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_4);
	/* USER CODE END ADC1_2_IRQn 1 */
}

/**
 * @brief This function handles USB high priority interrupt remap.
 */
void USB_HP_IRQHandler(void)
{
	/* USER CODE BEGIN USB_HP_IRQn 0 */

	/* USER CODE END USB_HP_IRQn 0 */
	HAL_PCD_IRQHandler(&hpcd_USB_FS);
	/* USER CODE BEGIN USB_HP_IRQn 1 */

	/* USER CODE END USB_HP_IRQn 1 */
}

/**
 * @brief This function handles USB low priority interrupt remap.
 */
void USB_LP_IRQHandler(void)
{
	/* USER CODE BEGIN USB_LP_IRQn 0 */

	/* USER CODE END USB_LP_IRQn 0 */
	HAL_PCD_IRQHandler(&hpcd_USB_FS);
	/* USER CODE BEGIN USB_LP_IRQn 1 */

	/* USER CODE END USB_LP_IRQn 1 */
}

/**
 * @brief This function handles HRTIM timer F global interrupt.
 */
void HRTIM1_TIMF_IRQHandler(void)
{
	/* USER CODE BEGIN HRTIM1_TIMF_IRQn 0 */
	LL_HRTIM_ClearFlag_REP(HRTIM1, LL_HRTIM_TIMER_F);

	vinRawVolt = (vinRawADC * VDDA) / 0xFFF0;
	Vin = vinRawVolt / (33.0 / 1033.0);
	//Vin = Vin * VIN_CORRECTION_FACTOR;
	VinAverage = Moving_Average_Compute(Vin, &VinFilter);

	voutRawVolt = (voutRawADC * VDDA) / 0xFFF0;
	Vout = voutRawVolt / (33.0 / 1033.0);
	//Vout = Vout * VOUT_CORRECTION_FACTOR;
	VoutAverage = Moving_Average_Compute(Vout, &VoutFilter);

	curInRawVolt = (curInRawADC * VDDA) / 0xFFF0;
	CurIn = (curInRawVolt / 200.0) / 0.0004293;
	//CurIn = CurIn * CURIN_CORRECTION_FACTOR;
	CurInAverage = Moving_Average_Compute(CurIn, &CurInFilter);

	curOutRawVolt = (curOutRawADC * VDDA) / 0xFFF0;
	CurOut = (curOutRawVolt / 200.0) / 0.0004293;
	//CurOut = CurOut * CUROUT_CORRECTION_FACTOR;
	CurOutAverage = Moving_Average_Compute(CurOut, &CurOutFilter);

	//Choose Conversion Mode
	if (targetVout > (VinAverage + buckBoostBand))
	{
		conversionState = CONVERSION_STATE_BOOST;
		buckBoostBand = BUCK_BOOST_BAND - BUCK_BOOST_BAND_HYSTERESIS;
	}
	else if (targetVout < (VinAverage - buckBoostBand))
	{
		conversionState = CONVERSION_STATE_BUCK;
		buckBoostBand = BUCK_BOOST_BAND - BUCK_BOOST_BAND_HYSTERESIS;
	}
	else if (abs((int) (VinAverage - targetVout)) < buckBoostBand)
	{
		conversionState = CONVERSION_STATE_BUCKBOOST;
		buckBoostBand = BUCK_BOOST_BAND + BUCK_BOOST_BAND_HYSTERESIS;
	}

	//Limit Operating Voltage
	if (Vin >= OVER_VOLTAGE_PROTECTION || Vout >= OVER_VOLTAGE_PROTECTION)
	{
		conversionState = CONVERSION_STATE_SHUTDOWN;
		CurrentDuty = 0;
	}

	//if (prevConversionState != conversionState)
	//{
	//	CurrentDuty = LOWER_DC_LIMIT_BUCKBOOST;
	//}

	prevConversionState = conversionState;

	switch (conversionState)
	{
	case CONVERSION_STATE_BUCKBOOST:
		//Activate Outputs
		LL_HRTIM_EnableOutput(HRTIM1,
		LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TE2 | LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TF2);

		//Control Algorithm

		if (Vout < targetVout || CurInAverage <= 60)
		{
			CurrentDuty += (abs((int) Vout - targetVout) / 100);
		}
		if (Vout > targetVout && CurInAverage >= 60)
		{
			CurrentDuty -= (abs((int) Vout - targetVout) / 100);
		}

		//Update the computed duty cycle

		//Timer E: PCB Location : Right : Vin  : Buck Node
		//buck node active
		LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_E, constrain(CurrentDuty, UPPER_DC_LIMIT_BUCKBOOST,
		LOWER_DC_LIMIT_BUCKBOOST));

		//Timer F: PCB Location : Left  : Vout : Boost Node
		//boost node active
		LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_F, constrain(CurrentDuty, UPPER_DC_LIMIT_BUCKBOOST,
		LOWER_DC_LIMIT_BUCKBOOST));

		//Timer F Compare 3 for ADC Trigger, set to half of duty cycle to reduce noise
		LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_F, constrain(CurrentDuty / 2, UPPER_DC_LIMIT_BUCKBOOST / 2,
		LOWER_DC_LIMIT_BUCKBOOST / 2));

		//Get current Duty Cycle
		CurrentDuty = LL_HRTIM_TIM_GetCompare1(HRTIM1, LL_HRTIM_TIMER_E);
		break;

	case CONVERSION_STATE_BUCK:
		//Activate Outputs
		LL_HRTIM_EnableOutput(HRTIM1,
		LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TE2 | LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TF2);

		//Control Algorithm

		if (Vout < targetVout || CurInAverage <= 60)
		{
			CurrentDuty += (abs((int) Vout - targetVout) / 100);
		}
		if (Vout > targetVout && CurInAverage >= 60)
		{
			CurrentDuty -= (abs((int) Vout - targetVout) / 100);
		}

		//Update the computed duty cycle

		//Timer E: PCB Location : Right : Vin  : Buck Node
		//Always switching at the higher voltage node, buck node active
		LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_E, constrain(CurrentDuty, UPPER_DC_LIMIT_BUCK,
		LOWER_DC_LIMIT_BUCK));

		//Timer F: PCB Location : Left  : Vout : Boost Node
		//Activate high side switch permanently
		LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_F, 0);

		//Timer F Compare 3 for ADC Trigger, set to half of duty cycle to reduce noise
		LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_F, constrain(CurrentDuty / 2, UPPER_DC_LIMIT_BUCK / 2,
		LOWER_DC_LIMIT_BUCK / 2));

		//Get current Duty Cycle
		CurrentDuty = LL_HRTIM_TIM_GetCompare1(HRTIM1, LL_HRTIM_TIMER_E);
		break;
	case CONVERSION_STATE_BOOST:
		//Activate Outputs
		LL_HRTIM_EnableOutput(HRTIM1,
		LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TE2 | LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TF2);

		//Control Algorithm
		if (Vout < targetVout || CurInAverage <= 60)
		{
			CurrentDuty += (abs((int) Vout - targetVout) / 100);
		}
		if (Vout > targetVout && CurInAverage >= 60)
		{
			CurrentDuty -= (abs((int) Vout - targetVout) / 100);
		}

		//Update the computed duty cycle

		//Timer F: PCB Location : Left  : Vout : Boost Node
		//Always switching at the higher voltage node, boost node active
		LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_F, constrain(CurrentDuty, UPPER_DC_LIMIT_BOOST,
		LOWER_DC_LIMIT_BOOST));

		//Timer E: PCB Location : Right : Vin  : Buck Node
		//Activate high side switch permanently
		LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_E, PWM_PERIOD + 1);

		//Timer F Compare 3 for ADC Trigger, set to half of duty cycle to reduce noise
		LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_F, constrain(CurrentDuty / 2, UPPER_DC_LIMIT_BOOST / 2,
		LOWER_DC_LIMIT_BOOST / 2));

		//Get current Duty Cycle
		CurrentDuty = LL_HRTIM_TIM_GetCompare1(HRTIM1, LL_HRTIM_TIMER_F);
		break;
	case CONVERSION_STATE_SHUTDOWN:
	default:
		LL_HRTIM_DisableOutput(HRTIM1,
		LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TE2 | LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TF2);
		break;
	}

	/* USER CODE END HRTIM1_TIMF_IRQn 0 */

	/* USER CODE BEGIN HRTIM1_TIMF_IRQn 1 */

	/* USER CODE END HRTIM1_TIMF_IRQn 1 */
}

/* USER CODE BEGIN 1 */
inline uint32_t constrain(uint32_t input, uint32_t upperLimit, uint32_t lowerLimit)
{
	return input > lowerLimit ? (input < upperLimit ? input : upperLimit) : lowerLimit;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
