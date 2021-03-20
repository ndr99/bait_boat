#include "servo_control.h"

//1000us <- -90deg, 2000us <- 90deg (linear)
//from servo angle to pwm delay
int angle_to_us(int deg, int T_ON_min, int T_ON_max, int phi_min, int phi_max){
    int m = (T_ON_min-T_ON_max)/(phi_min-phi_max);
    int b = T_ON_min-m*phi_min;
    int delay_time = m * deg + b;
    delay_time = min_int(max_int(delay_time,T_ON_min), T_ON_max);
    return delay_time;
}

//pwm config function with pin and freq
ledc_channel_config_t PWM_config(int pin, uint32_t freq, ledc_timer_t timer, ledc_channel_t channel, ledc_timer_bit_t duty_resolution){
    //at first the timer config
    const ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
        .duty_resolution = duty_resolution, // resolution of PWM duty
        .timer_num = timer,            // timer index
        .freq_hz = freq,                      // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    //then channel config 
    const ledc_channel_config_t PWM_config_t = {
        pin,
        LEDC_HIGH_SPEED_MODE,
        channel, 
        LEDC_INTR_DISABLE, 
        timer,
        0,
        0
    };
    ledc_channel_config(&PWM_config_t);
    return PWM_config_t;
}

//Servo T_on to duty cycle
double duty_cycle_from_T_on_calc(int T_on, int freq){
    return (double)T_on * (double)freq / 10e5;
}

//0-1 (double) range duty cycle to 0-12bit (int) range duty cycle to output
int duty_cycle_int_calc(double duty_cycle, int duty_resolution){
    return (int)(duty_cycle * (double)(1<<duty_resolution));
}


//pass duty cycle to pin
void set_duty(int duty_cycle, ledc_channel_config_t PWM_config_t){
    ledc_set_duty(PWM_config_t.speed_mode, PWM_config_t.channel, duty_cycle);
    ledc_update_duty(PWM_config_t.speed_mode, PWM_config_t.channel);
}