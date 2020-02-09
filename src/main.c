#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include <math.h>

#define DC_PWM_PIN 5
#define DC_PWM_FREQ 25000
#define DC_DUTY_RESOLUTION LEDC_TIMER_11_BIT
#define STEERING_SERVO_PIN 4
#define STEERING_SERVO_FREQ 50
#define STEERING_DUTY_RESOLUTION LEDC_TIMER_12_BIT



// handle/ref to the blink task to delete it
TaskHandle_t blink_task_handle = NULL;

// blink_task prototype
void blink_task();

// minmax prototype
int max(int x, int y);
int min(int x, int y);

//angle to mikroseconds prototype
int angle_to_us(int deg);

//triangle
int triangle_wave(int i, int maxvalue);

// Do something for 1 microseconds
void wait_one_micros();

// do something for delay_us microseconds
void wait_micros(int delay_us);

//PWM_config prototype
ledc_channel_config_t PWM_config(int pin, int freq, int timer, int channel, int duty_resolution);

//duty_cycle calc prototype
double duty_cycle_from_T_on_calc(int T_on, int freq);

//convert duty cycle to int format duty cycle for output
int duty_cycle_int_calc(double duty_cycle, int duty_resolution);

//set_duty prototype
void set_duty(int duty_cycle, ledc_channel_config_t PWM_config_t);


void app_main(){
    printf("hi\n");
    //create the task to blink the led
    xTaskCreate(
        &blink_task, //memory address
        "blink_task", //name of the task
        2048, //bits of the stack
        NULL, //parameter to pass (empty)
        4, //priority
        blink_task_handle //handle/ref to the blink task to delete it
        );
}


//blink the led 30times
void blink_task(){

    //set up the steering servo
    ledc_channel_config_t steering_PWM_config = PWM_config(
        STEERING_SERVO_PIN,
        STEERING_SERVO_FREQ,
        LEDC_TIMER_0,
        LEDC_CHANNEL_0,
        STEERING_DUTY_RESOLUTION
    );
    
    ledc_channel_config_t DC_PWM_config = PWM_config(
        DC_PWM_PIN,
        DC_PWM_FREQ,
        LEDC_TIMER_1,
        LEDC_CHANNEL_1,
        DC_DUTY_RESOLUTION
    );

    printf("Start PWM\n");
    int i = 0;
    int servo_angle = 0;
    int number_of_sections = 36; // 180deg/5deg
    int T_on = 0;
    double duty_cycle = 0;
    int duty_cycle_int = 0;

    int throttle = 0;
    int max_percentage = 40;
    double DC_duty_cycle = 0;
    int DC_duty_cycle_int = 0;

    while(1){

        // Calculate and output Servo motor PWM
        //calculate the servo angle from triangle wave
        servo_angle = 5 * triangle_wave(i, number_of_sections);
        printf("Servo angle: %d\n", servo_angle);

        //calculate the T_on in microseconds
        T_on = angle_to_us(servo_angle);
        printf("T_on: %d\n", T_on);

        //calculate the duty cycle
        duty_cycle = duty_cycle_from_T_on_calc(T_on, STEERING_SERVO_FREQ);
        printf("Servo duty double: %lf\n", duty_cycle);

        //convert double duty cycle to int duty cycle
        duty_cycle_int = duty_cycle_int_calc(duty_cycle, STEERING_DUTY_RESOLUTION);
        printf("Servo duty int: %d\n", duty_cycle_int);

        //pass to pin
        set_duty(duty_cycle_int, steering_PWM_config);


        // Calculate and output DC motor PWM
        //calculate throttle from triangle wave
        throttle = triangle_wave(i, max_percentage) + 80;
        printf("Throttle: %d\n", throttle);

        //calculate duty cycle from throttle
        DC_duty_cycle = throttle / 100.;
        printf("DC_duty_cycle: %lf\n", DC_duty_cycle);

        DC_duty_cycle_int = duty_cycle_int_calc(DC_duty_cycle, DC_DUTY_RESOLUTION);
        printf("DC_duty_cycle_int: %d\n", DC_duty_cycle_int);

        //pass to pin
        set_duty(DC_duty_cycle_int, DC_PWM_config);

        vTaskDelay(100 / portTICK_PERIOD_MS);
        i++;
    }
    printf("Delete task\n");
    vTaskDelete(blink_task_handle);
}


//1000us <- -90deg, 2000us <- 90deg (linear)
//from servo angle to pwm delay
int angle_to_us(int deg){
    int m = 500 / 90;
    int b = 1500;
    int delay_time = m * deg + b;
    delay_time = min(max(delay_time,1000), 2000);
    return delay_time;
}


int min(int x, int y){
    if(x<y){
        return x;
    }else{
        return y;
    }
}


int max(int x, int y){
    if(x>y){
        return x;
    }else{
        return y;
    }
}


//triangle wave divided into number of steps
int triangle_wave(int i, int number_of_steps){
    return abs(i % (2 * number_of_steps) - number_of_steps)-number_of_steps/2;
}


// Do something for 1 microseconds
void wait_one_micros(){
    for(volatile int k=0;k<7;k++){};
}


// do something for delay_us microseconds
void wait_micros(int delay_us){
    for(int j=0;j<delay_us;j++){
        wait_one_micros();
    }
}


 int gpio_num;                   /*!< the LEDC output gpio_num, if you want to use gpio16, gpio_num = 16 */
    ledc_mode_t speed_mode;         /*!< LEDC speed speed_mode, high-speed mode or low-speed mode */
    ledc_channel_t channel;         /*!< LEDC channel (0 - 7) */
    ledc_intr_type_t intr_type;     /*!< configure interrupt, Fade interrupt enable  or Fade interrupt disable */
    ledc_timer_t timer_sel;         /*!< Select the timer source of channel (0 - 3) */
    uint32_t duty;                  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
    int hpoint;                     /*!< LEDC channel hpoint value, the max value is 0xfffff */

//pwm config function with pin and freq
ledc_channel_config_t PWM_config(int pin, int freq, int timer, int channel, int duty_resolution){
    //at first the timer config
    const ledc_timer_config_t ledc_timer = {
        .duty_resolution = duty_resolution, // resolution of PWM duty
        .freq_hz = freq,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
        .timer_num = timer,            // timer index
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