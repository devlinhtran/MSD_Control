#ifndef READ_INPUT_SIGNAL_H
#define READ_INPUT_SIGNAL_H

typedef struct Signal_input_info
{
    uint8_t mode_run;
    uint8_t duty_cycle_pwm_in;
    uint8_t last_duty_cycle_pwm_in;
    uint8_t en_pwm;
    uint8_t last_en_pwm;
    uint8_t analog_signal_in;
    uint8_t last_analog_signal_in;
    uint8_t dir;
    uint8_t last_dir;
    uint64_t time_on_pwm;
    uint64_t time_off_pwm;
    uint64_t start_time_rise;
    uint64_t start_time_fall;
    uint64_t start_time_change_pwm_in;
}signal_input_info_t;

#define RCC_HIGH    2000
#define RCC_LOW     1000
#define RCC_MID     1500
#define RCC_DELTA   0

void read_input_pwm_signal(uint8_t id);
void read_input_analog_signal(uint8_t id);
void process_input_signal(void);
void cal_duty_cycle_pwm_in(void);
void init_input_info(void);
void process_input_signal_motor(uint8_t id);
void read_input_rc_signal(uint8_t id);
void read_dir_in(void);
void reset_motor_info(uint8_t id);

#endif
