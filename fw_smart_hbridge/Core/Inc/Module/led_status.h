#ifndef LED_STATUS_H
#define LED_STATUS_H

typedef struct Led_info
{
    uint8_t led_level;      //0: led off, 1: led on
    uint8_t en_blink;       //0: led normal, 1: led blink
    uint32_t start_time;    //count time to generate blink signal
    uint32_t period;        //Period for blink mode 0.05ms per unit
    uint8_t step;           //Step for blink mode
}led_info_t;

enum{
    LED_OFF = 0,
    LED_ON,
    BLINK_ENABLE,
    BLINK_DISABLE
};

typedef struct Led_data
{
    led_info_t led_run;
    led_info_t led_error;
    led_info_t led_over_current;
}led_data_t;

void control_led_run(uint8_t id);
void control_led_error(uint8_t id);
void control_led_over_current(uint8_t id);
void control_all_led(void);
void control_led_status(void);
void control_led_motor_status(uint8_t id);
#endif
