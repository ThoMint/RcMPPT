/**
  ******************************************************************************
  * @file    hrtim.c
  * @brief   This file provides code for the configuration
  *          of the HRTIM instances.
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

/* Includes ------------------------------------------------------------------*/
#include "hrtim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_HRTIM1);

  /* HRTIM1 interrupt Init */
  NVIC_SetPriority(HRTIM1_TIMF_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(HRTIM1_TIMF_IRQn);

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  LL_HRTIM_ConfigDLLCalibration(HRTIM1, LL_HRTIM_DLLCALIBRATION_MODE_CONTINUOUS, LL_HRTIM_DLLCALIBRATION_RATE_3);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 100; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_ConfigADCTrig(HRTIM1, LL_HRTIM_ADCTRIG_2, LL_HRTIM_ADCTRIG_UPDATE_TIMER_F, LL_HRTIM_ADCTRIG_SRC24_TIMFCMP3);
  LL_HRTIM_SetADCPostScaler(HRTIM1, LL_HRTIM_ADCTRIG_2, 0x0);
  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_PRESCALERRATIO_MUL32);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_E, PWM_PERIOD);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_E, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_COUNTING_MODE_UP);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_E, 10880);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_DT_PRESCALER_MUL8);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_E, 68);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_E, 68);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_E, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TE1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TE1, LL_HRTIM_OUTPUTSET_TIMPER);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TE1, LL_HRTIM_OUTPUTRESET_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TE1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TE1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TE1, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TE1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TE2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TE2, LL_HRTIM_OUTPUTSET_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TE2, LL_HRTIM_OUTPUTRESET_NONE);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TE2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TE2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TE2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TE2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 100; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_PRESCALERRATIO_MUL32);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_F, PWM_PERIOD);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_F, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_COUNTING_MODE_UP);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetComp3Mode(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_GTCMP3_EQUAL);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_EnableDLYPRT(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_F, 10880);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_F, 5000);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_DT_PRESCALER_MUL8);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_F, 68);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_F, 68);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_F, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TF1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TF1, LL_HRTIM_OUTPUTSET_TIMCMP1);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TF1, LL_HRTIM_OUTPUTRESET_TIMPER);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TF1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TF1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TF1, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TF1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TF2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TF2, LL_HRTIM_OUTPUTSET_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TF2, LL_HRTIM_OUTPUTRESET_NONE);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TF2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TF2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TF2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TF2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    /**HRTIM1 GPIO Configuration
    PC6     ------> HRTIM1_CHF1
    PC7     ------> HRTIM1_CHF2
    PC8     ------> HRTIM1_CHE1
    PC9     ------> HRTIM1_CHE2
    */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
