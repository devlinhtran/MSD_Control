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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32g0xx_hal.h"
#include "adc.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./Module/button.h"
#include "./Module/control_motor.h"
#include "./Module/led_status.h"
#include "./Module/read_adc.h"
#include "./Module/read_input_signal.h"
#include "./Module/ringbuf.h"
#include "./Module/serial.h"
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
#define HBR1_LED_RUN_Pin GPIO_PIN_13
#define HBR1_LED_RUN_GPIO_Port GPIOC
#define HBR2_LED_RUN_Pin GPIO_PIN_14
#define HBR2_LED_RUN_GPIO_Port GPIOC
#define PWM5_OUT_FAN_Pin GPIO_PIN_0
#define PWM5_OUT_FAN_GPIO_Port GPIOF
#define Temperature_Pin GPIO_PIN_0
#define Temperature_GPIO_Port GPIOA
#define Vbus_Feed_Pin GPIO_PIN_2
#define Vbus_Feed_GPIO_Port GPIOA
#define HBR1_iLIMIT_Pin GPIO_PIN_3
#define HBR1_iLIMIT_GPIO_Port GPIOA
#define HBR2_iLIMIT_Pin GPIO_PIN_4
#define HBR2_iLIMIT_GPIO_Port GPIOA
#define HBR1_iSEN_Pin GPIO_PIN_5
#define HBR1_iSEN_GPIO_Port GPIOA
#define HBR2_iSEN_Pin GPIO_PIN_6
#define HBR2_iSEN_GPIO_Port GPIOA
#define BR2_PWM1_Pin GPIO_PIN_0
#define BR2_PWM1_GPIO_Port GPIOB
#define BR2_PWM2_Pin GPIO_PIN_1
#define BR2_PWM2_GPIO_Port GPIOB
#define HBR1_ACCE_Pin GPIO_PIN_2
#define HBR1_ACCE_GPIO_Port GPIOB
#define HBR2_ACCE_Pin GPIO_PIN_10
#define HBR2_ACCE_GPIO_Port GPIOB
#define HBR1_PROTENTION_Pin GPIO_PIN_11
#define HBR1_PROTENTION_GPIO_Port GPIOB
#define HBR2_PROTENTION_Pin GPIO_PIN_12
#define HBR2_PROTENTION_GPIO_Port GPIOB
#define HBR2_LED_OVER_CUR_Pin GPIO_PIN_13
#define HBR2_LED_OVER_CUR_GPIO_Port GPIOB
#define HBR2_LED_ERR_Pin GPIO_PIN_14
#define HBR2_LED_ERR_GPIO_Port GPIOB
#define HBR2_PWM_IN_Pin GPIO_PIN_8
#define HBR2_PWM_IN_GPIO_Port GPIOA
#define HBR2_PWM_IN_EXTI_IRQn EXTI4_15_IRQn
#define HBR1_PWM_IN_Pin GPIO_PIN_9
#define HBR1_PWM_IN_GPIO_Port GPIOA
#define HBR1_PWM_IN_EXTI_IRQn EXTI4_15_IRQn
#define BR1_PWM1_Pin GPIO_PIN_6
#define BR1_PWM1_GPIO_Port GPIOC
#define BR1_PWM2_Pin GPIO_PIN_7
#define BR1_PWM2_GPIO_Port GPIOC
#define HBR1_DIR_IN_Pin GPIO_PIN_10
#define HBR1_DIR_IN_GPIO_Port GPIOA
#define HBR2_DIR_IN_Pin GPIO_PIN_11
#define HBR2_DIR_IN_GPIO_Port GPIOA
#define UART1_DE_Pin GPIO_PIN_12
#define UART1_DE_GPIO_Port GPIOA
#define BUTTON_CH2_TEST_Pin GPIO_PIN_15
#define BUTTON_CH2_TEST_GPIO_Port GPIOA
#define BUTTON_CH1_TEST_Pin GPIO_PIN_0
#define BUTTON_CH1_TEST_GPIO_Port GPIOD
#define HBR1_STATUS_Pin GPIO_PIN_1
#define HBR1_STATUS_GPIO_Port GPIOD
#define HBR2_REST_Pin GPIO_PIN_2
#define HBR2_REST_GPIO_Port GPIOD
#define HBR2_STATUS_Pin GPIO_PIN_3
#define HBR2_STATUS_GPIO_Port GPIOD
#define HBR1_REST_Pin GPIO_PIN_3
#define HBR1_REST_GPIO_Port GPIOB
#define HBR1_LED_OVER_CUR_Pin GPIO_PIN_4
#define HBR1_LED_OVER_CUR_GPIO_Port GPIOB
#define HBR1_LED_ERR_Pin GPIO_PIN_5
#define HBR1_LED_ERR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
enum{
    HBR1 = 0,
    HBR2
};
enum{
    BUTTON_1 = 0,
    BUTTON_2
};
enum{
    PWM_SIGNAL_INPUT = 0,
    RC_SIGNAL_INPUT
};
extern button_info_t button_info[2];
extern led_data_t led_data[2];
extern signal_input_info_t signal_input_info[2];
extern adc_data_t adc_data;
extern motor_run_info_t motor_info[2];
extern adc_convert_data_t adc_convert_data;

extern uint8_t config_pwm_pin_mode;
extern uint64_t timer_tick_pwm_in;              //Timer 17 24MHZ
extern uint32_t os_time;    //Timer 16 20KHz 0.05ms per unit
extern uint8_t err_code;
extern uint8_t first_power_on;
extern uint32_t start_led_power_on;

extern uint8_t data_uart;
extern RINGBUF sRingBuffer;
extern uint8_t serialRxBuffer[SERIAL_BUFFER_LENGH];
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
