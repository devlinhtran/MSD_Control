#ifndef CONTROL_MOTOR_H
#define CONTROL_MOTOR_H

typedef struct Motor_run_info
{
    uint8_t mode;
    uint8_t last_mode;
    uint8_t dir;
    uint8_t duty_cycle;
    uint16_t acce;          //percent adc acce
    uint16_t acce_tick;     // = acce * vel /100
    uint16_t vel;           // = duty_cycle * vel_max /100
    uint16_t vel_current;
    uint32_t time_change_vel;
    uint32_t start_change;
    uint32_t start_change_dir;

}motor_run_info_t;

enum{
    MOTOR_IDLE = 0,
    MOTOR_RUN_PWM_INPUT,
    MOTOR_RUN_ANALOG_INPUT,
    MOTOR_RUN_TEST_BUTTON_1,
    MOTOR_RUN_TEST_BUTTON_2,
    MOTOR_STOP,
    MOTOR_RUN_UART_INPUT,
    MOTOR_RUN_RC_INPUT,
    MOTOR_PRE_CHANGE_DIR
};

#define VELOCITY_MAX   2399
#define TIME_CHANGE_VELOCITY 2000   //100ms

void run_motor(uint8_t id, motor_run_info_t motor_info);
void stop_motor(uint8_t id);
void control_run_motor(uint8_t id);
void control_run_all_motor(void);
#endif
