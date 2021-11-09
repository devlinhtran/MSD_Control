#include "main.h"

uint8_t pin_hbr1_pwm, pin_hbr2_pwm, pin_hbr1_pwm_last, pin_hbr2_pwm_last, step_hbr1 = 0, step_hbr2 = 0;


void read_input_pwm_signal(uint8_t id)
{
    uint8_t pin_val = 0;
    if(id == HBR1)
    {
        signal_input_info[id].en_pwm = pinRead(HBR1_STATUS_GPIO_Port, HBR1_STATUS_Pin);
        signal_input_info[id].dir = pinRead(HBR1_DIR_IN_GPIO_Port, HBR1_DIR_IN_Pin);
        if(timer_tick_pwm_in - signal_input_info[id].start_time_change_pwm_in > 24000) //not change the pwm for 1ms
        {
            pin_val = pinRead(HBR1_PWM_IN_GPIO_Port, HBR1_PWM_IN_Pin);
            if(pin_val == 0)
            {
                signal_input_info[id].duty_cycle_pwm_in = 0;
            }
            else
            {
                signal_input_info[id].duty_cycle_pwm_in = 100;
            }
        }
        else if(signal_input_info[id].time_on_pwm != 0 && signal_input_info[id].time_off_pwm != 0)
        {
            signal_input_info[id].duty_cycle_pwm_in \
                    = signal_input_info[id].time_on_pwm/(signal_input_info[id].time_on_pwm + signal_input_info[id].time_off_pwm)*100;
        }
        if(signal_input_info[id].time_on_pwm != 0 && signal_input_info[id].time_off_pwm != 0\
                && (signal_input_info[id].time_on_pwm + signal_input_info[id].time_off_pwm > 24000)\
                && (signal_input_info[id].time_on_pwm + signal_input_info[id].time_off_pwm < 240))
        {
            signal_input_info[id].duty_cycle_pwm_in = 0;
            err_code = ERROR_PWM_OUT_OF_RANGE_MOTOR_1;
        }
    }

    if(id == HBR2)
    {
        signal_input_info[id].en_pwm = pinRead(HBR2_STATUS_GPIO_Port, HBR2_STATUS_Pin);
        signal_input_info[id].dir = pinRead(HBR2_DIR_IN_GPIO_Port, HBR2_DIR_IN_Pin);
        if(timer_tick_pwm_in - signal_input_info[id].start_time_change_pwm_in > 24000) //not change the pwm for 1ms
        {
            pin_val = pinRead(HBR2_PWM_IN_GPIO_Port, HBR2_PWM_IN_Pin);
            if(pin_val == 0)
            {
                signal_input_info[id].duty_cycle_pwm_in = 0;
            }
            else
            {
                signal_input_info[id].duty_cycle_pwm_in = 100;
            }
        }
        else if(signal_input_info[id].time_on_pwm != 0 && signal_input_info[id].time_off_pwm != 0)
        {
            signal_input_info[id].duty_cycle_pwm_in \
                    = signal_input_info[id].time_on_pwm/(signal_input_info[id].time_on_pwm + signal_input_info[id].time_off_pwm)*100;
        }
        if(signal_input_info[id].time_on_pwm != 0 && signal_input_info[id].time_off_pwm != 0\
                && (signal_input_info[id].time_on_pwm + signal_input_info[id].time_off_pwm > 24000)\
                && (signal_input_info[id].time_on_pwm + signal_input_info[id].time_off_pwm < 240))
        {
            signal_input_info[id].duty_cycle_pwm_in = 0;
            err_code = ERROR_PWM_OUT_OF_RANGE_MOTOR_2;
        }
    }
}


void read_input_rc_signal(uint8_t id)
{
    if(id == HBR1)
    {
        signal_input_info[id].en_pwm = pinRead(HBR1_STATUS_GPIO_Port, HBR1_STATUS_Pin);
        if(timer_tick_pwm_in - signal_input_info[id].start_time_change_pwm_in > 12000000) //not change the pwm for 500ms
        {
            signal_input_info[id].duty_cycle_pwm_in = 0;
        }
        else if(signal_input_info[id].time_on_pwm != 0 && signal_input_info[id].time_off_pwm != 0)
        {
            if((signal_input_info[id].time_on_pwm > (RCC_MID + RCC_DELTA)) && (signal_input_info[id].time_on_pwm < RCC_HIGH))
            {
                signal_input_info[id].duty_cycle_pwm_in \
                        = (signal_input_info[id].time_on_pwm - RCC_MID)/(RCC_HIGH - RCC_MID)*100;
                signal_input_info[id].dir = 0;
            }
            else if((signal_input_info[id].time_on_pwm < (RCC_MID - RCC_DELTA)) && (signal_input_info[id].time_on_pwm > RCC_LOW))
            {
                signal_input_info[id].duty_cycle_pwm_in \
                        = (RCC_MID - signal_input_info[id].time_on_pwm)/(RCC_MID - RCC_LOW)*100;
                signal_input_info[id].dir = 1;
            }
            else if((signal_input_info[id].time_on_pwm < (RCC_MID - RCC_DELTA)) && (signal_input_info[id].time_on_pwm > (RCC_MID + RCC_DELTA)))
            {
                signal_input_info[id].duty_cycle_pwm_in = 0;
            }
            else
            {
                signal_input_info[id].duty_cycle_pwm_in = 0;
                err_code = ERROR_RC_OUT_OF_RANGE_MOTOR_1;
            }
        }
    }

    if(id == HBR2)
    {
        signal_input_info[id].en_pwm = pinRead(HBR2_STATUS_GPIO_Port, HBR2_STATUS_Pin);
        if(timer_tick_pwm_in - signal_input_info[id].start_time_change_pwm_in > 12000000) //not change the pwm for 1ms
        {
            signal_input_info[id].duty_cycle_pwm_in = 0;
        }
        else if(signal_input_info[id].time_on_pwm != 0 && signal_input_info[id].time_off_pwm != 0)
        {
            if((signal_input_info[id].time_on_pwm > (RCC_MID + RCC_DELTA)) && (signal_input_info[id].time_on_pwm < RCC_HIGH))
            {
                signal_input_info[id].duty_cycle_pwm_in \
                        = (signal_input_info[id].time_on_pwm - RCC_MID)/(RCC_HIGH - RCC_MID)*100;
                signal_input_info[id].dir = 0;
            }
            else if((signal_input_info[id].time_on_pwm < (RCC_MID - RCC_DELTA)) && (signal_input_info[id].time_on_pwm > RCC_LOW))
            {
                signal_input_info[id].duty_cycle_pwm_in \
                        = (RCC_MID - signal_input_info[id].time_on_pwm)/(RCC_MID - RCC_LOW)*100;
                signal_input_info[id].dir = 1;
            }
            else if((signal_input_info[id].time_on_pwm < (RCC_MID - RCC_DELTA)) && (signal_input_info[id].time_on_pwm > (RCC_MID + RCC_DELTA)))
            {
                signal_input_info[id].duty_cycle_pwm_in = 0;
            }
            else
            {
                signal_input_info[id].duty_cycle_pwm_in = 0;
                err_code = ERROR_RC_OUT_OF_RANGE_MOTOR_2;
            }
        }
    }
}

void read_input_analog_signal(uint8_t id)
{
    if(id == HBR1)
    {
        signal_input_info[id].dir = pinRead(HBR1_DIR_IN_GPIO_Port, HBR1_DIR_IN_Pin);
        signal_input_info[id].analog_signal_in \
                = adc_data.adc_HBR1_Protention*100/4096;
    }

    if(id == HBR2)
    {
        signal_input_info[id].dir = pinRead(HBR2_DIR_IN_GPIO_Port, HBR2_DIR_IN_Pin);
        signal_input_info[id].analog_signal_in \
                = adc_data.adc_HBR2_Protention*100/4096;
    }
}

void init_input_info(void)
{
    pin_hbr1_pwm_last = pinRead(HBR1_PWM_IN_GPIO_Port, HBR1_PWM_IN_Pin);
    pin_hbr2_pwm_last = pinRead(HBR2_PWM_IN_GPIO_Port, HBR2_PWM_IN_Pin);
    signal_input_info[HBR1].last_dir = pinRead(HBR1_DIR_IN_GPIO_Port, HBR1_DIR_IN_Pin);
    signal_input_info[HBR2].last_dir = pinRead(HBR2_DIR_IN_GPIO_Port, HBR2_DIR_IN_Pin);
}

//Calculate the pwm in signal of motor
//The function call in external gpio interrupt function
void cal_duty_cycle_pwm_in(void)
{
    pin_hbr1_pwm = pinRead(HBR1_PWM_IN_GPIO_Port, HBR1_PWM_IN_Pin);
    pin_hbr2_pwm = pinRead(HBR2_PWM_IN_GPIO_Port, HBR2_PWM_IN_Pin);
    //Process HBR1 motor input signal
    if(pin_hbr1_pwm !=  pin_hbr1_pwm_last)
    {
        signal_input_info[HBR1].start_time_change_pwm_in = timer_tick_pwm_in;
        pin_hbr1_pwm_last = pin_hbr1_pwm;
        switch (step_hbr1) {
        case 0:
            if(pin_hbr1_pwm)
            {
                signal_input_info[HBR1].start_time_rise = timer_tick_pwm_in;
                step_hbr1 = 1;
            }
            break;
        case 1:
            if(pin_hbr1_pwm == 0)
            {
                signal_input_info[HBR1].start_time_fall = timer_tick_pwm_in;
                signal_input_info[HBR1].time_on_pwm = timer_tick_pwm_in - signal_input_info[HBR1].start_time_rise;
                step_hbr1 = 2;
            }
            break;
        case 2:
            if(pin_hbr1_pwm)
            {
                signal_input_info[HBR1].time_off_pwm = timer_tick_pwm_in - signal_input_info[HBR1].start_time_fall;
                step_hbr1 = 0;
            }
            break;
        default:
            step_hbr1 = 0;
            signal_input_info[HBR1].time_on_pwm = 0;
            signal_input_info[HBR1].time_off_pwm = 0;
            break;
        }
    }

    //Process HBR2 motor input signal
    if(pin_hbr2_pwm !=  pin_hbr2_pwm_last)
    {
        signal_input_info[HBR2].start_time_change_pwm_in = timer_tick_pwm_in;
        pin_hbr2_pwm_last = pin_hbr2_pwm;
        switch (step_hbr2) {
        case 0:
            if(pin_hbr2_pwm)
            {
                signal_input_info[HBR2].start_time_rise = timer_tick_pwm_in;
                step_hbr2 = 1;
            }
            break;
        case 1:
            if(pin_hbr2_pwm == 0)
            {
                signal_input_info[HBR2].start_time_fall = timer_tick_pwm_in;
                signal_input_info[HBR2].time_on_pwm = timer_tick_pwm_in - signal_input_info[HBR2].start_time_rise;
                step_hbr2 = 2;
            }
            break;
        case 2:
            if(pin_hbr2_pwm)
            {
                signal_input_info[HBR2].time_off_pwm = timer_tick_pwm_in - signal_input_info[HBR2].start_time_fall;
                step_hbr2 = 0;
            }
            break;
        default:
            step_hbr2 = 0;
            signal_input_info[HBR2].time_on_pwm = 0;
            signal_input_info[HBR2].time_off_pwm = 0;
            break;
        }
    }

}

void read_dir_in(void)
{
    signal_input_info[HBR1].dir = pinRead(HBR1_DIR_IN_GPIO_Port, HBR1_DIR_IN_Pin);
    signal_input_info[HBR2].dir = pinRead(HBR2_DIR_IN_GPIO_Port, HBR2_DIR_IN_Pin);
    if(signal_input_info[HBR1].dir != signal_input_info[HBR1].last_dir)
    {
        signal_input_info[HBR1].last_dir = signal_input_info[HBR1].dir;
        if(motor_info[HBR1].mode == MOTOR_RUN_ANALOG_INPUT || motor_info[HBR1].mode == MOTOR_RUN_PWM_INPUT)
        {
            motor_info[HBR1].last_mode = motor_info[HBR1].mode;
            motor_info[HBR1].mode = MOTOR_PRE_CHANGE_DIR;
            motor_info[HBR1].dir = signal_input_info[HBR1].dir;
        }
    }

    if(signal_input_info[HBR2].dir != signal_input_info[HBR2].last_dir)
    {
        signal_input_info[HBR2].last_dir = signal_input_info[HBR2].dir;
        if(motor_info[HBR2].mode == MOTOR_RUN_ANALOG_INPUT || motor_info[HBR2].mode == MOTOR_RUN_PWM_INPUT)
        {
            motor_info[HBR1].last_mode = motor_info[HBR2].mode;
            motor_info[HBR2].mode = MOTOR_PRE_CHANGE_DIR;
            motor_info[HBR2].dir = signal_input_info[HBR2].dir;
        }
    }

}

void process_input_signal_motor(uint8_t id)
{
    if(motor_info[id].mode != MOTOR_RUN_TEST_BUTTON_1 && motor_info[id].mode != MOTOR_RUN_TEST_BUTTON_2\
            && motor_info[id].mode != MOTOR_STOP && motor_info[id].mode != MOTOR_PRE_CHANGE_DIR)
    {
        if(config_pwm_pin_mode == PWM_SIGNAL_INPUT)
        {
            read_input_pwm_signal(id);
        }
        else if(config_pwm_pin_mode == RC_SIGNAL_INPUT)
        {
            read_input_rc_signal(id);
        }
        read_input_analog_signal(id);

        if(config_pwm_pin_mode == PWM_SIGNAL_INPUT)
        {
            if((signal_input_info[id].duty_cycle_pwm_in != signal_input_info[id].last_duty_cycle_pwm_in\
                || (motor_info[id].mode == MOTOR_IDLE && (motor_info[id].last_mode == MOTOR_IDLE || motor_info[id].last_mode == MOTOR_RUN_PWM_INPUT))) \
                    && signal_input_info[id].en_pwm != 0)
            {
                motor_info[id].mode = MOTOR_RUN_PWM_INPUT;
                motor_info[id].duty_cycle = signal_input_info[id].duty_cycle_pwm_in;
                signal_input_info[id].last_duty_cycle_pwm_in = signal_input_info[id].duty_cycle_pwm_in;
                motor_info[id].vel = motor_info[id].duty_cycle * 2399 / 100;
                motor_info[id].dir = signal_input_info[id].dir;
            }
        }
        else if(config_pwm_pin_mode == RC_SIGNAL_INPUT)
        {
            if((signal_input_info[id].duty_cycle_pwm_in != signal_input_info[id].last_duty_cycle_pwm_in\
                || (motor_info[id].mode == MOTOR_IDLE && (motor_info[id].last_mode == MOTOR_IDLE || motor_info[id].last_mode == MOTOR_RUN_RC_INPUT))) \
                    && signal_input_info[id].en_pwm != 0)
            {
                motor_info[id].mode = MOTOR_RUN_RC_INPUT;
                motor_info[id].duty_cycle = signal_input_info[id].duty_cycle_pwm_in;
                signal_input_info[id].last_duty_cycle_pwm_in = signal_input_info[id].duty_cycle_pwm_in;
                motor_info[id].vel = motor_info[id].duty_cycle * 2399 / 100;
                motor_info[id].dir = signal_input_info[id].dir;
            }
        }

        if((signal_input_info[id].analog_signal_in != signal_input_info[id].last_analog_signal_in\
            || (motor_info[id].mode == MOTOR_IDLE && (motor_info[id].last_mode == MOTOR_IDLE || motor_info[id].last_mode == MOTOR_RUN_ANALOG_INPUT))) \
                && signal_input_info[id].en_pwm != 0)
        {
            motor_info[id].mode = MOTOR_RUN_ANALOG_INPUT;
            motor_info[id].duty_cycle = signal_input_info[id].analog_signal_in;
            signal_input_info[id].last_analog_signal_in = signal_input_info[id].analog_signal_in;
            motor_info[id].vel = motor_info[id].duty_cycle * 2399 / 100;
            motor_info[id].dir = signal_input_info[id].dir;
        }

        if(motor_info[id].mode == MOTOR_RUN_ANALOG_INPUT || motor_info[id].mode == MOTOR_RUN_PWM_INPUT || motor_info[id].mode == MOTOR_RUN_RC_INPUT)
        {
            if(signal_input_info[id].en_pwm == 0)
            {
                reset_motor_info(id);
            }
        }
    }
}

void process_input_signal(void)
{
    process_input_signal_motor(HBR1);
    process_input_signal_motor(HBR2);
}

void reset_motor_info(uint8_t id)
{
    motor_info[id].last_mode = motor_info[id].mode;
    motor_info[id].mode = MOTOR_IDLE;
    motor_info[id].duty_cycle = 0;
    motor_info[id].vel = 0;
    motor_info[id].dir = 0;
}
