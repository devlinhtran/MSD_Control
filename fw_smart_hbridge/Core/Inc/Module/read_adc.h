#ifndef READ_ADC_H
#define READ_ADC_H

typedef struct Adc_data
{
    uint32_t adc_tem;
    uint32_t adc_VbusFeed;
    uint32_t adc_HBR1_iLimit;
    uint32_t adc_HBR2_iLimit;
    uint32_t adc_HBR1_iSen;
    uint32_t adc_HBR2_iSen;
    uint32_t adc_HBR1_Acce;
    uint32_t adc_HBR2_Acce;
    uint32_t adc_HBR1_Protention;
    uint32_t adc_HBR2_Protention;
    uint32_t time_wait_read_20Hz;
    uint32_t time_wait_read_1KHz;

}adc_data_t;

typedef struct Adc_convert_data
{
    uint32_t adc_tem;
    uint32_t adc_VbusFeed;
    uint32_t adc_HBR1_iLimit;
    uint32_t adc_HBR2_iLimit;
    uint32_t adc_HBR1_iSen;
    uint32_t adc_HBR2_iSen;
    uint32_t adc_HBR1_Acce;
    uint32_t adc_HBR2_Acce;
    uint32_t adc_HBR1_Protention;
    uint32_t adc_HBR2_Protention;
    uint8_t  has_read_adc_tem;                 //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_VbusFeed;            //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR1_iLimit;         //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR2_iLimit;         //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR1_iSen;           //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR2_iSen;           //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR1_Acce;           //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR2_Acce;           //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR1_Protention;     //1: The adc value is valid, 0: The adc value is invalid
    uint8_t  has_read_adc_HBR2_Protention;     //1: The adc value is valid, 0: The adc value is invalid
    uint32_t time_wait_read;
    uint8_t  counter_adc_timeout;
}adc_convert_data_t;

enum error_code{
    NORMAL = 0,
    ERROR_ALERT_TEMPERATURE,
    ERROR_ALERT_OVER_CURRENT_MOTOR_1,
    ERROR_ALERT_OVER_CURRENT_MOTOR_2,
    ERROR_ALERT_OVER_CURRENT_BOARD,
    ERROR_TIMEOUT_ADC,
    ERROR_RC_OUT_OF_RANGE_MOTOR_1,
    ERROR_RC_OUT_OF_RANGE_MOTOR_2,
    ERROR_PWM_OUT_OF_RANGE_MOTOR_1,
    ERROR_PWM_OUT_OF_RANGE_MOTOR_2,


};

#define TIME_READ_ADC_20Hz  1000 //50ms
#define TIME_READ_ADC_1KHz  20 //1ms

#define TEMPERATURE_LIMIT 4096
#define BOARD_CURRENT_LIMIT 4096

void convert_all_adc_value(void);
uint8_t process_error(void);
void convert_adc_value_1KHz(void);
void convert_adc_value_20Hz(void);
#endif
