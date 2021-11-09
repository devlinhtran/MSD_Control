/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, HBR1_LED_RUN_Pin|HBR2_LED_RUN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HBR2_LED_OVER_CUR_Pin|HBR2_LED_ERR_Pin|HBR1_REST_Pin|HBR1_LED_OVER_CUR_Pin
                          |HBR1_LED_ERR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART1_DE_GPIO_Port, UART1_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HBR2_REST_GPIO_Port, HBR2_REST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = HBR1_LED_RUN_Pin|HBR2_LED_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = HBR2_LED_OVER_CUR_Pin|HBR2_LED_ERR_Pin|HBR1_REST_Pin|HBR1_LED_OVER_CUR_Pin
                          |HBR1_LED_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = HBR2_PWM_IN_Pin|HBR1_PWM_IN_Pin|HBR1_DIR_IN_Pin|HBR2_DIR_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = BUTTON_CH2_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = UART1_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART1_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = BUTTON_CH1_TEST_Pin|HBR1_STATUS_Pin|HBR2_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = HBR2_REST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HBR2_REST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */
void pinMode(GPIO_TypeDef  *GPIOx, uint32_t pin, uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_DEFAULT;

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void pinHighZ(GPIO_TypeDef *GPIOx, uint32_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_DEFAULT;

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void pinOutputLow(GPIO_TypeDef *GPIOx, uint32_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_DEFAULT;

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
}

void pinOutputHigh(GPIO_TypeDef *GPIOx, uint32_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_DEFAULT;

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
}

void pinLow(GPIO_TypeDef *GPIOx, uint32_t pin)
{
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
}

void pinHigh(GPIO_TypeDef *GPIOx, uint32_t pin)
{
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
}

void pinWrite(GPIO_TypeDef *GPIOx, uint32_t pin, uint8_t val)
{
    if(val)
    {
        pinHigh(GPIOx, pin);
    }
    else
    {
        pinLow(GPIOx, pin);
    }
}

uint8_t pinRead(GPIO_TypeDef *GPIOx, uint32_t pin)
{
    return (HAL_GPIO_ReadPin(GPIOx, pin));
}



/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
