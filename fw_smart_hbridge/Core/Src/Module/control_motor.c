#include "main.h"

void run_motor(uint8_t id, motor_run_info_t motor_info)
{
//    uint16_t time_counter = motor_info.duty_cycle * 2400/100;
    if(id == HBR1)
    {
        if(motor_info.dir == 0)
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            htim3.Instance->CCR1 = motor_info.vel_current;
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

        }

        if(motor_info.dir == 1)
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
            htim3.Instance->CCR2 = motor_info.vel_current;
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

        }
    }

    if(id == HBR2)
    {
        if(motor_info.dir == 0)
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
            htim3.Instance->CCR3 = motor_info.vel_current;
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

        }

        if(motor_info.dir == 1)
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
            htim3.Instance->CCR4 = motor_info.vel_current;
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

        }
    }
}

void stop_motor(uint8_t id)
{
    if(id == HBR1)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    }

    if(id == HBR2)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    }
}

void control_run_motor(uint8_t id)
{
    motor_info[id].acce_tick = motor_info[id].vel * motor_info[id].acce / 100;
    motor_info[id].time_change_vel = TIME_CHANGE_VELOCITY; //100ms
    if(motor_info[id].acce_tick == 0)
    {
        motor_info[id].acce_tick = 1;
    }
    switch (motor_info[id].mode) {
    case MOTOR_IDLE:
        if(os_time - motor_info[id].start_change > motor_info[id].time_change_vel)
        {
            if(motor_info[id].vel_current > motor_info[id].acce_tick)
            {
                motor_info[id].vel_current = motor_info[id].vel_current - motor_info[id].acce_tick;
                motor_info[id].start_change = os_time;
            }
            else
            {
                motor_info[id].vel_current = 0;
            }
        }
        run_motor(id, motor_info[id]);
        break;
    case MOTOR_RUN_PWM_INPUT:
    case MOTOR_RUN_ANALOG_INPUT:
    case MOTOR_RUN_RC_INPUT:
        if(os_time - motor_info[id].start_change > motor_info[id].time_change_vel)
        {
            if(motor_info[id].vel_current > motor_info[id].acce_tick + motor_info[id].vel)
            {
                motor_info[id].vel_current = motor_info[id].vel_current - motor_info[id].acce_tick;
                motor_info[id].start_change = os_time;
            }
            else if(motor_info[id].vel_current < motor_info[id].vel - motor_info[id].acce_tick)
            {
                motor_info[id].vel_current = motor_info[id].vel_current + motor_info[id].acce_tick;
                motor_info[id].start_change = os_time;
            }
            else if((motor_info[id].vel_current < (motor_info[id].vel + motor_info[id].acce_tick))\
                    && (motor_info[id].vel_current > (motor_info[id].vel - motor_info[id].acce_tick)))
            {
                motor_info[id].vel_current = motor_info[id].vel;
                motor_info[id].start_change = os_time;
            }
        }
        run_motor(id, motor_info[id]);
        break;
    case MOTOR_RUN_TEST_BUTTON_1:
        if(os_time - motor_info[id].start_change > motor_info[id].time_change_vel)
        {
            motor_info[id].dir = 0;
            if(motor_info[id].vel_current > motor_info[id].acce_tick + (VELOCITY_MAX * 80/100))
            {
                motor_info[id].vel_current = motor_info[id].vel_current - motor_info[id].acce_tick;
                motor_info[id].start_change = os_time;
            }
            else if(motor_info[id].vel_current < (VELOCITY_MAX * 80/100) - motor_info[id].acce)
            {
                motor_info[id].vel_current = motor_info[id].vel_current + motor_info[id].acce_tick;
                motor_info[id].start_change = os_time;
            }
            else if((motor_info[id].vel_current < ((VELOCITY_MAX * 80/100) + motor_info[id].acce))\
                    && (motor_info[id].vel_current > ((VELOCITY_MAX * 80/100) - motor_info[id].acce)))
            {
                motor_info[id].vel_current = (VELOCITY_MAX * 80/100);
                motor_info[id].start_change = os_time;
            }
        }
        run_motor(id, motor_info[id]);
        break;
    case MOTOR_RUN_TEST_BUTTON_2:
        if(os_time - motor_info[id].start_change > motor_info[id].time_change_vel)
        {
            motor_info[id].dir = 1;
            if(motor_info[id].vel_current > motor_info[id].acce_tick + (VELOCITY_MAX * 80/100))
            {
                motor_info[id].vel_current = motor_info[id].vel_current - motor_info[id].acce_tick;
                motor_info[id].start_change = os_time;
            }
            else if(motor_info[id].vel_current < (VELOCITY_MAX * 80/100) - motor_info[id].acce)
            {
                motor_info[id].vel_current = motor_info[id].vel_current + motor_info[id].acce_tick;
                motor_info[id].start_change = os_time;
            }
            else if((motor_info[id].vel_current < ((VELOCITY_MAX * 80/100) + motor_info[id].acce))\
                    && (motor_info[id].vel_current > ((VELOCITY_MAX * 80/100) - motor_info[id].acce)))
            {
                motor_info[id].vel_current = (VELOCITY_MAX * 80/100);
                motor_info[id].start_change = os_time;
            }
        }
        run_motor(id, motor_info[id]);
        break;
    case MOTOR_STOP:
        stop_motor(id);
        motor_info[id].vel_current  =0;
        break;
    case MOTOR_PRE_CHANGE_DIR:
        stop_motor(id);
        if(os_time - motor_info[id].start_change_dir > 2000)//100ms
        {
            motor_info[id].mode = motor_info[id].last_mode;
            motor_info[id].vel_current = 0;
        }
        break;
    default:
        break;
    }

}

void control_run_all_motor(void)
{
    control_run_motor(HBR1);
    control_run_motor(HBR2);

}


