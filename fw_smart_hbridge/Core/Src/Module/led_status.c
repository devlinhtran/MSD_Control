#include "main.h"



void control_led_run(uint8_t id)
{
    if(id == HBR1)
    {
        if(led_data[id].led_run.led_level == LED_ON)
        {
            if(led_data[id].led_run.en_blink == BLINK_ENABLE)
            {
                switch (led_data[id].led_run.step) {
                case 0:
                    pinHigh(HBR1_LED_RUN_GPIO_Port, HBR1_LED_RUN_Pin);
                    led_data[id].led_run.start_time = os_time;
                    led_data[id].led_run.step = 1;
                    break;
                case 1:
                    if(os_time - led_data[id].led_run.start_time > (led_data[id].led_run.period/2))
                    {
                        pinLow(HBR1_LED_RUN_GPIO_Port, HBR1_LED_RUN_Pin);
                        led_data[id].led_run.start_time = os_time;
                        led_data[id].led_run.step = 2;
                    }
                    break;
                case 2:
                    if(os_time - led_data[id].led_run.start_time > (led_data[id].led_run.period/2))
                    {
                        led_data[id].led_run.step = 0;
                    }
                    break;
                default:
                    led_data[id].led_run.step = 0;
                    break;
                }
            }
            else
            {
                pinHigh(HBR1_LED_RUN_GPIO_Port, HBR1_LED_RUN_Pin);
            }
        }
        else
        {
            pinLow(HBR1_LED_RUN_GPIO_Port, HBR1_LED_RUN_Pin);
        }
    }
    else if(id == HBR2)
    {
        if(led_data[id].led_run.led_level == LED_ON)
        {
            if(led_data[id].led_run.en_blink == BLINK_ENABLE)
            {
                switch (led_data[id].led_run.step) {
                case 0:
                    pinHigh(HBR2_LED_RUN_GPIO_Port, HBR2_LED_RUN_Pin);
                    led_data[id].led_run.start_time = os_time;
                    led_data[id].led_run.step = 1;
                    break;
                case 1:
                    if(os_time - led_data[id].led_run.start_time > (led_data[id].led_run.period/2))
                    {
                        pinLow(HBR2_LED_RUN_GPIO_Port, HBR2_LED_RUN_Pin);
                        led_data[id].led_run.start_time = os_time;
                        led_data[id].led_run.step = 2;
                    }
                    break;
                case 2:
                    if(os_time - led_data[id].led_run.start_time > (led_data[id].led_run.period/2))
                    {
                        led_data[id].led_run.step = 0;
                    }
                    break;
                default:
                    led_data[id].led_run.step = 0;
                    break;
                }
            }
            else
            {
                pinHigh(HBR2_LED_RUN_GPIO_Port, HBR2_LED_RUN_Pin);
            }
        }
        else
        {
            pinLow(HBR2_LED_RUN_GPIO_Port, HBR2_LED_RUN_Pin);
        }
    }
}
void control_led_error(uint8_t id)
{
    if(id == HBR1)
    {
        if(led_data[id].led_error.led_level == LED_ON)
        {
            if(led_data[id].led_error.en_blink == BLINK_ENABLE)
            {
                switch (led_data[id].led_error.step) {
                case 0:
                    pinHigh(HBR1_LED_ERR_GPIO_Port, HBR1_LED_ERR_Pin);
                    led_data[id].led_error.start_time = os_time;
                    led_data[id].led_error.step = 1;
                    break;
                case 1:
                    if(os_time - led_data[id].led_error.start_time > (led_data[id].led_error.period/2))
                    {
                        pinLow(HBR1_LED_ERR_GPIO_Port, HBR1_LED_ERR_Pin);
                        led_data[id].led_error.start_time = os_time;
                        led_data[id].led_error.step = 2;
                    }
                    break;
                case 2:
                    if(os_time - led_data[id].led_error.start_time > (led_data[id].led_error.period/2))
                    {
                        led_data[id].led_error.step = 0;
                    }
                    break;
                default:
                    led_data[id].led_error.step = 0;
                    break;
                }
            }
            else
            {
                pinHigh(HBR1_LED_ERR_GPIO_Port, HBR1_LED_ERR_Pin);
            }
        }
        else
        {
            pinLow(HBR1_LED_ERR_GPIO_Port, HBR1_LED_ERR_Pin);
        }
    }
    else if(id == HBR2)
    {
        if(led_data[id].led_error.led_level == LED_ON)
        {
            if(led_data[id].led_error.en_blink == BLINK_ENABLE)
            {
                switch (led_data[id].led_error.step) {
                case 0:
                    pinHigh(HBR2_LED_ERR_GPIO_Port, HBR2_LED_ERR_Pin);
                    led_data[id].led_error.start_time = os_time;
                    led_data[id].led_error.step = 1;
                    break;
                case 1:
                    if(os_time - led_data[id].led_error.start_time > (led_data[id].led_error.period/2))
                    {
                        pinLow(HBR2_LED_ERR_GPIO_Port, HBR2_LED_ERR_Pin);
                        led_data[id].led_error.start_time = os_time;
                        led_data[id].led_error.step = 2;
                    }
                    break;
                case 2:
                    if(os_time - led_data[id].led_error.start_time > (led_data[id].led_error.period/2))
                    {
                        led_data[id].led_error.step = 0;
                    }
                    break;
                default:
                    led_data[id].led_error.step = 0;
                    break;
                }
            }
            else
            {
                pinHigh(HBR2_LED_ERR_GPIO_Port, HBR2_LED_ERR_Pin);
            }
        }
        else
        {
            pinLow(HBR2_LED_ERR_GPIO_Port, HBR2_LED_ERR_Pin);
        }
    }
}
void control_led_over_current(uint8_t id)
{
    if(id == HBR1)
    {
        if(led_data[id].led_over_current.led_level == LED_ON)
        {
            if(led_data[id].led_over_current.en_blink == BLINK_ENABLE)
            {
                switch (led_data[id].led_over_current.step) {
                case 0:
                    pinHigh(HBR1_LED_OVER_CUR_GPIO_Port, HBR1_LED_OVER_CUR_Pin);
                    led_data[id].led_over_current.start_time = os_time;
                    led_data[id].led_over_current.step = 1;
                    break;
                case 1:
                    if(os_time - led_data[id].led_over_current.start_time > (led_data[id].led_over_current.period/2))
                    {
                        pinLow(HBR1_LED_OVER_CUR_GPIO_Port, HBR1_LED_OVER_CUR_Pin);
                        led_data[id].led_over_current.start_time = os_time;
                        led_data[id].led_over_current.step = 2;
                    }
                    break;
                case 2:
                    if(os_time - led_data[id].led_over_current.start_time > (led_data[id].led_over_current.period/2))
                    {
                        led_data[id].led_over_current.step = 0;
                    }
                    break;
                default:
                    led_data[id].led_over_current.step = 0;
                    break;
                }
            }
            else
            {
                pinHigh(HBR1_LED_OVER_CUR_GPIO_Port, HBR1_LED_OVER_CUR_Pin);
            }
        }
        else
        {
            pinLow(HBR1_LED_OVER_CUR_GPIO_Port, HBR1_LED_OVER_CUR_Pin);
        }
    }
    else if(id == HBR2)
    {
        if(led_data[id].led_over_current.led_level == LED_ON)
        {
            if(led_data[id].led_over_current.en_blink == BLINK_ENABLE)
            {
                switch (led_data[id].led_over_current.step) {
                case 0:
                    pinHigh(HBR2_LED_OVER_CUR_GPIO_Port, HBR2_LED_OVER_CUR_Pin);
                    led_data[id].led_over_current.start_time = os_time;
                    led_data[id].led_over_current.step = 1;
                    break;
                case 1:
                    if(os_time - led_data[id].led_over_current.start_time > (led_data[id].led_over_current.period/2))
                    {
                        pinLow(HBR2_LED_OVER_CUR_GPIO_Port, HBR2_LED_OVER_CUR_Pin);
                        led_data[id].led_over_current.start_time = os_time;
                        led_data[id].led_over_current.step = 2;
                    }
                    break;
                case 2:
                    if(os_time - led_data[id].led_over_current.start_time > (led_data[id].led_over_current.period/2))
                    {
                        led_data[id].led_over_current.step = 0;
                    }
                    break;
                default:
                    led_data[id].led_over_current.step = 0;
                    break;
                }
            }
            else
            {
                pinHigh(HBR2_LED_OVER_CUR_GPIO_Port, HBR2_LED_OVER_CUR_Pin);
            }
        }
        else
        {
            pinLow(HBR2_LED_OVER_CUR_GPIO_Port, HBR2_LED_OVER_CUR_Pin);
        }
    }
}

void control_all_led(void)
{
    control_led_error(HBR1);
    control_led_over_current(HBR1);
    control_led_run(HBR1);

    control_led_error(HBR2);
    control_led_over_current(HBR2);
    control_led_run(HBR2);
}

void control_led_motor_status(uint8_t id)
{

    if(motor_info[id].mode != MOTOR_IDLE && motor_info[id].mode != MOTOR_STOP)
    {
        led_data[id].led_run.led_level = LED_OFF;
        led_data[id].led_run.en_blink = BLINK_DISABLE;
    }
    else if(motor_info[id].mode == MOTOR_RUN_ANALOG_INPUT)
    {
        led_data[id].led_run.led_level = LED_ON;
        led_data[id].led_run.en_blink = BLINK_DISABLE;
    }
    else if(motor_info[id].mode == MOTOR_RUN_PWM_INPUT)
    {
        led_data[id].led_run.led_level = LED_ON;
        led_data[id].led_run.en_blink = BLINK_ENABLE;
        led_data[id].led_over_current.period = 40000; //0.5Hz 2s
    }
    else if(motor_info[id].mode == MOTOR_RUN_UART_INPUT)
    {
        led_data[id].led_run.led_level = LED_ON;
        led_data[id].led_run.en_blink = BLINK_ENABLE;
        led_data[id].led_over_current.period = 10000; //2Hz 0.5s
    }
    else if(motor_info[id].mode == MOTOR_RUN_RC_INPUT)
    {
        led_data[id].led_run.led_level = LED_ON;
        led_data[id].led_run.en_blink = BLINK_ENABLE;
        led_data[id].led_over_current.period = 20000; //1Hz 1s
    }

    if(err_code == ERROR_ALERT_OVER_CURRENT_BOARD)
    {
        led_data[id].led_over_current.led_level = LED_ON;
        led_data[id].led_over_current.en_blink = BLINK_DISABLE;
    }
    if(err_code == ERROR_ALERT_OVER_CURRENT_MOTOR_1)
    {
        led_data[id].led_over_current.led_level = LED_ON;
        led_data[id].led_over_current.en_blink = BLINK_ENABLE;
        led_data[id].led_over_current.period = 20000; //1Hz 1s
    }
    if(err_code == ERROR_ALERT_TEMPERATURE || err_code == ERROR_TIMEOUT_ADC)
    {
        led_data[id].led_error.led_level = LED_ON;
        led_data[id].led_error.en_blink = BLINK_DISABLE;
    }
    if(err_code == NORMAL)
    {
        led_data[id].led_error.led_level = LED_OFF;
        led_data[id].led_over_current.led_level = LED_OFF;
    }
}

void control_led_status(void)
{
    control_led_motor_status(HBR1);
    control_led_motor_status(HBR2);

    if(first_power_on == 1)// Led run on 1s when power on board
    {
        led_data[HBR1].led_run.led_level = LED_ON;
        led_data[HBR1].led_run.en_blink = BLINK_DISABLE;
        led_data[HBR2].led_run.led_level = LED_ON;
        led_data[HBR2].led_run.en_blink = BLINK_DISABLE;
        if(start_led_power_on == 0)
            start_led_power_on = os_time;
        if(os_time - start_led_power_on > 20000) //1s
        {
            led_data[HBR1].led_run.led_level = LED_OFF;
            led_data[HBR1].led_run.en_blink = BLINK_DISABLE;
            led_data[HBR2].led_run.led_level = LED_OFF;
            led_data[HBR2].led_run.en_blink = BLINK_DISABLE;
            first_power_on = 0;
        }
    }
}
