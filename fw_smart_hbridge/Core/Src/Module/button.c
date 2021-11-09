#include "main.h"

//Read button status
//id: 0: Button 1, 1: Button 2
uint8_t read_button(uint8_t id)
{
    uint8_t result = 0;
    switch (id) {
    case 0:
        result = pinRead(BUTTON_CH1_TEST_GPIO_Port, BUTTON_CH1_TEST_Pin);
        break;
    case 1:
        result = pinRead(BUTTON_CH2_TEST_GPIO_Port, BUTTON_CH2_TEST_Pin);
        break;
    default:
        break;
    }
    return result;
}

//Read Button Pin and check if hold the button or double press the button
//id: BUTTON_1 or BUTTON_2
void process_read_button(uint8_t id)
{
    uint8_t pin_tmp = read_button(id);
    if(button_info[id].pin_status != pin_tmp)
    {
        button_info[id].start_time = os_time;
        button_info[id].count_button = 1;
        button_info[id].pin_status = pin_tmp;
    }

    if(button_info[id].count_button > 0 && (os_time - button_info[id].start_time) > TIME_COUNT_BUTTON)
    {
        button_info[id].count_button++;
        button_info[id].start_time = os_time;

    }

    if(button_info[id].count_button > COUNT_NUMBER_CHANGE_BUTTON_STATUS)
    {
        button_info[id].count_button = 0;
        if(button_info[id].pin_status == 0)
        {
            button_info[id].press_status = BUTTON_PRESSED;
            button_info[id].start_time_pressed = os_time;
        }
        else
        {
            button_info[id].press_status = BUTTON_RELEASED;
            button_info[id].start_time_released = os_time;

        }
    }


    if(((os_time - button_info[id].start_time_pressed) > TIME_DETECT_PRESSED_HOLD)\
            && button_info[id].press_status == BUTTON_PRESSED)
    {
        button_info[id].press_status = BUTTON_HOLD;
    }

}

//Process Button 1 and 2 with Hold Button and Double Press Button case
//Hold Button -> Motor mode: MOTOR_RUN_TEST_BUTTON_1 or MOTOR_RUN_TEST_BUTTON_2
//Press both button 500ms -> Software reset the IC
void process_button(void)
{
    unsigned char id = 1;
    process_read_button(BUTTON_1);
    process_read_button(BUTTON_2);

    id = BUTTON_1;

    if(motor_info[HBR1].mode != MOTOR_RUN_TEST_BUTTON_2\
            && motor_info[HBR2].mode != MOTOR_RUN_TEST_BUTTON_2)
    {
        if(button_info[id].press_status == BUTTON_HOLD\
                && motor_info[HBR1].mode != MOTOR_STOP\
                && motor_info[HBR2].mode != MOTOR_STOP)
        {
            if(motor_info[HBR2].mode != MOTOR_RUN_TEST_BUTTON_2\
                    && motor_info[HBR1].mode != MOTOR_RUN_TEST_BUTTON_2\
                    && motor_info[HBR1].mode != MOTOR_RUN_TEST_BUTTON_1\
                    && motor_info[HBR2].mode != MOTOR_RUN_TEST_BUTTON_1)
            {
                motor_info[HBR2].last_mode = motor_info[HBR2].mode;
                motor_info[HBR1].last_mode = motor_info[HBR1].mode;
            }

            motor_info[HBR1].mode = MOTOR_RUN_TEST_BUTTON_1;
            motor_info[HBR2].mode = MOTOR_RUN_TEST_BUTTON_1;
        }
        else
        {
            motor_info[HBR1].mode = motor_info[HBR1].last_mode;
            motor_info[HBR2].mode = motor_info[HBR2].last_mode;
        }
    }

    id = BUTTON_2;

    if(motor_info[HBR1].mode != MOTOR_RUN_TEST_BUTTON_1\
            && motor_info[HBR2].mode != MOTOR_RUN_TEST_BUTTON_1)
    {
        if(button_info[id].press_status == BUTTON_HOLD\
                && motor_info[HBR1].mode != MOTOR_STOP\
                && motor_info[HBR2].mode != MOTOR_STOP)
        {
            if(motor_info[HBR2].mode != MOTOR_RUN_TEST_BUTTON_2\
                    && motor_info[HBR1].mode != MOTOR_RUN_TEST_BUTTON_2\
                    && motor_info[HBR1].mode != MOTOR_RUN_TEST_BUTTON_1\
                    && motor_info[HBR2].mode != MOTOR_RUN_TEST_BUTTON_1)
            {
                motor_info[HBR2].last_mode = motor_info[HBR2].mode;
                motor_info[HBR1].last_mode = motor_info[HBR1].mode;
            }
            motor_info[HBR1].mode = MOTOR_RUN_TEST_BUTTON_2;
            motor_info[HBR2].mode = MOTOR_RUN_TEST_BUTTON_2;
        }
        else
        {
            motor_info[HBR1].mode = motor_info[HBR1].last_mode;
            motor_info[HBR2].mode = motor_info[HBR2].last_mode;
        }
    }

    //Process pressed both button.
    if((button_info[BUTTON_1].press_status == BUTTON_HOLD || button_info[BUTTON_1].press_status == BUTTON_PRESSED)\
            && (button_info[BUTTON_2].press_status == BUTTON_HOLD || button_info[BUTTON_2].press_status == BUTTON_PRESSED))
    {
        if(os_time - button_info[BUTTON_1].start_time_both_button_pressed > TIME_DETECT_PRESSED_BOTH)
        {
            button_info[BUTTON_1].start_time_both_button_pressed = os_time;
            button_info[BUTTON_1].count_both_button++;
            if(button_info[BUTTON_1].count_both_button>5)
            {
                HAL_NVIC_SystemReset();
            }
        }
    }
    else
    {
        button_info[BUTTON_1].count_both_button = 0;
        button_info[BUTTON_2].count_both_button = 0;

    }

}
