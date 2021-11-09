#ifndef BUTTON_H
#define BUTTON_H

typedef struct Button_info
{
    uint8_t pin_status;
    uint8_t press_status;
    uint8_t last_press_status;

    uint32_t start_time;
    uint32_t start_time_pressed;
    uint32_t start_time_released;
    uint8_t count_button;
    uint32_t start_time_both_button_pressed;
    uint8_t count_both_button;
}button_info_t;


#define TIME_DETECT_PRESSED_HOLD 10000  //0.5s
#define TIME_DETECT_PRESSED_BOTH 2000  //0.1s
#define TIME_DETECT_DOUBLE_PRESS 60000  //3s
#define TIME_COUNT_BUTTON 200           //10ms
#define COUNT_NUMBER_CHANGE_BUTTON_STATUS 5

enum{
    BUTTON_IDLE = 0,
    BUTTON_PRESSED,
    BUTTON_RELEASED,
    BUTTON_HOLD,
    BUTTON_DOUBLE_PRESS
};

uint8_t read_button(uint8_t id);
void process_read_button(uint8_t id);
void process_button(void);

#endif
