/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
#define GPIO_SPEED_DEFAULT      GPIO_SPEED_FREQ_VERY_HIGH

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */void pinMode(GPIO_TypeDef  *GPIOx, uint32_t pin, uint32_t mode, uint32_t pull);
void pinHighZ(GPIO_TypeDef *GPIOx, uint32_t pin);
void pinOutputLow(GPIO_TypeDef *GPIOx, uint32_t pin);
void pinOutputHigh(GPIO_TypeDef *GPIOx, uint32_t pin);
void pinLow(GPIO_TypeDef *GPIOx, uint32_t pin);
void pinHigh(GPIO_TypeDef *GPIOx, uint32_t pin);
void pinWrite(GPIO_TypeDef *GPIOx, uint32_t pin, uint8_t val);
uint8_t pinRead(GPIO_TypeDef *GPIOx, uint32_t pin);

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
