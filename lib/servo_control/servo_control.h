#include "driver/ledc.h"
#include "helper.h"

//angle to microseconds prototype
int angle_to_us(int deg, int T_ON_min, int T_ON_max, int phi_min, int phi_max);

//PWM_config prototype
ledc_channel_config_t PWM_config(int pin, uint32_t freq, ledc_timer_t timer, ledc_channel_t channel, ledc_timer_bit_t duty_resolution);

//duty_cycle calc prototype
double duty_cycle_from_T_on_calc(int T_on, int freq);

//convert duty cycle to int format duty cycle for output
int duty_cycle_int_calc(double duty_cycle, int duty_resolution);

//set_duty prototype
void set_duty(int duty_cycle, ledc_channel_config_t PWM_config_t);