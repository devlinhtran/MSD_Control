/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PB2     ------> ADC1_IN10
    PB10     ------> ADC1_IN11
    PB11     ------> ADC1_IN15
    PB12     ------> ADC1_IN16
    */
    GPIO_InitStruct.Pin = Temperature_Pin|Vbus_Feed_Pin|HBR1_iLIMIT_Pin|HBR2_iLIMIT_Pin
                          |HBR1_iSEN_Pin|HBR2_iSEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HBR1_ACCE_Pin|HBR2_ACCE_Pin|HBR1_PROTENTION_Pin|HBR2_PROTENTION_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PB2     ------> ADC1_IN10
    PB10     ------> ADC1_IN11
    PB11     ------> ADC1_IN15
    PB12     ------> ADC1_IN16
    */
    HAL_GPIO_DeInit(GPIOA, Temperature_Pin|Vbus_Feed_Pin|HBR1_iLIMIT_Pin|HBR2_iLIMIT_Pin
                          |HBR1_iSEN_Pin|HBR2_iSEN_Pin);

    HAL_GPIO_DeInit(GPIOB, HBR1_ACCE_Pin|HBR2_ACCE_Pin|HBR1_PROTENTION_Pin|HBR2_PROTENTION_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}


uint16_t read_adc_iLimit(uint8_t id)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    if(id == HBR1)
    {
        sConfig.Channel = ADC_CHANNEL_3;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR1_iLimit = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }

    if(id == HBR2)
    {
        sConfig.Channel = ADC_CHANNEL_4;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR2_iLimit = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }
    return 0;
}
uint16_t read_adc_iSen(uint8_t id)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    if(id == HBR1)
    {
        sConfig.Channel = ADC_CHANNEL_5;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR1_iSen = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }

    if(id == HBR2)
    {
        sConfig.Channel = ADC_CHANNEL_6;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR2_iSen = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }
    return 0;
}
uint16_t read_adc_iAcce(uint8_t id)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    if(id == HBR1)
    {
        sConfig.Channel = ADC_CHANNEL_10;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR1_Acce = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }

    if(id == HBR2)
    {
        sConfig.Channel = ADC_CHANNEL_11;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR2_Acce = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }
    return 0;
}
uint16_t read_adc_iAnalog_input(uint8_t id)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    if(id == HBR1)
    {
        sConfig.Channel = ADC_CHANNEL_15;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR1_Protention = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }

    if(id == HBR2)
    {
        sConfig.Channel = ADC_CHANNEL_16;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
        {
            adc_data.adc_HBR2_Protention = HAL_ADC_GetValue(&hadc1);
            return 1;
        }
    }
    return 0;
}
uint16_t read_adc_temperature(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
    {
        adc_data.adc_tem = HAL_ADC_GetValue(&hadc1);
        return 1;
    }
    return 0;
}

uint16_t read_adc_Vbus_feed(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
    {
        adc_data.adc_VbusFeed = HAL_ADC_GetValue(&hadc1);
        return 1;
    }
    return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
