#include "main.h"


void convert_adc_value_20Hz(void)
{
    if((os_time - adc_data.time_wait_read_20Hz) < TIME_READ_ADC_20Hz) //Wait 50ms to read again
        return;

    if(read_adc_Vbus_feed())
    {
        //Cal current Vbus
        adc_convert_data.has_read_adc_VbusFeed = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_VbusFeed = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_temperature())
    {
        //Cal temperature
        adc_convert_data.has_read_adc_tem = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_tem = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_iAcce(HBR1))
    {
        //Cal accelerate motor
        adc_convert_data.has_read_adc_HBR1_Acce = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR1_Acce = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_iLimit(HBR1))
    {
        //Cal current limit motor
        adc_convert_data.has_read_adc_HBR1_iLimit = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR1_iLimit = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_iAnalog_input(HBR1))
    {
        //Cal analog input signal
        adc_convert_data.has_read_adc_HBR1_Protention = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR1_Protention = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_iAcce(HBR2))
    {
        //Cal accelerate motor
        adc_convert_data.has_read_adc_HBR2_Acce = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR2_Acce = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_iLimit(HBR2))
    {
        //Cal current limit motor
        adc_convert_data.has_read_adc_HBR2_iLimit = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR2_iLimit = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_iAnalog_input(HBR2))
    {
        //Cal analog input signal
        adc_convert_data.has_read_adc_HBR2_Protention = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR2_Protention = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    adc_data.time_wait_read_20Hz = os_time;
}
void convert_adc_value_1KHz(void)
{
    if((os_time - adc_data.time_wait_read_1KHz) < TIME_READ_ADC_1KHz) //Wait 50ms to read again
        return;

    if(read_adc_iSen(HBR1))
    {
        //Cal current of running motor
        adc_convert_data.has_read_adc_HBR1_iSen = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR1_iSen = 0;
        adc_convert_data.counter_adc_timeout++;
    }

    if(read_adc_iSen(HBR2))
    {
        //Cal current of running motor
        adc_convert_data.has_read_adc_HBR2_iSen = 1;
    }
    else
    {
        adc_convert_data.has_read_adc_HBR2_iSen = 0;
        adc_convert_data.counter_adc_timeout++;
    }


    adc_data.time_wait_read_1KHz = os_time;

}

//Read and convert the adc to the valid value
void convert_all_adc_value(void)
{
    convert_adc_value_1KHz();
    convert_adc_value_20Hz();
}


//Process the adc value to alert error
uint8_t process_error(void)
{
   if(adc_convert_data.adc_HBR1_Protention > adc_convert_data.adc_HBR1_iLimit \
           && adc_convert_data.has_read_adc_HBR1_Protention == 1\
           && adc_convert_data.has_read_adc_HBR1_iLimit == 1)
   {
       err_code = ERROR_ALERT_OVER_CURRENT_MOTOR_1;
       motor_info[HBR1].mode = MOTOR_STOP;
       motor_info[HBR2].mode = MOTOR_STOP;

   }

   if(adc_convert_data.adc_HBR2_Protention > adc_convert_data.adc_HBR2_iLimit \
           && adc_convert_data.has_read_adc_HBR2_Protention == 1\
           && adc_convert_data.has_read_adc_HBR2_iLimit == 1)
   {
       err_code = ERROR_ALERT_OVER_CURRENT_MOTOR_2;
       motor_info[HBR1].mode = MOTOR_STOP;
       motor_info[HBR2].mode = MOTOR_STOP;
   }

   if(adc_convert_data.adc_tem > TEMPERATURE_LIMIT && adc_convert_data.has_read_adc_tem == 1)
   {
       err_code = ERROR_ALERT_TEMPERATURE;
       motor_info[HBR1].mode = MOTOR_STOP;
       motor_info[HBR2].mode = MOTOR_STOP;
   }
   else if(err_code == ERROR_ALERT_TEMPERATURE)
   {
       motor_info[HBR1].mode = MOTOR_IDLE;
       motor_info[HBR2].mode = MOTOR_IDLE;
       err_code = NORMAL;
   }

   if(adc_convert_data.adc_VbusFeed > BOARD_CURRENT_LIMIT && adc_convert_data.has_read_adc_VbusFeed == 1)
   {
       err_code = ERROR_ALERT_OVER_CURRENT_BOARD;
       motor_info[HBR1].mode = MOTOR_STOP;
       motor_info[HBR2].mode = MOTOR_STOP;
   }

   if(adc_convert_data.counter_adc_timeout > 10)
   {
       err_code = ERROR_TIMEOUT_ADC;
   }
   return err_code;

}
