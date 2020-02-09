#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include <math.h>
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_wifi_types.h"
#include <string.h>
#include "esp_http_server.h"
#include "tcpip_adapter.h"

#define DC_PWM_PIN 5
#define DC_PWM_FREQ 25000
#define DC_DUTY_RESOLUTION LEDC_TIMER_11_BIT
#define STEERING_SERVO_PIN 4
#define STEERING_SERVO_FREQ 50
#define STEERING_DUTY_RESOLUTION LEDC_TIMER_12_BIT

// WIFI ap authentication parameters
#define EXAMPLE_ESP_WIFI_SSID      "BAIT_BOAT"
#define EXAMPLE_ESP_WIFI_PASS      "barmilehet"
#define EXAMPLE_MAX_STA_CONN       1

// handle/ref to the motor control task to delete it
TaskHandle_t motor_control_task_handle = NULL;

// motor_control_task prototype
void motor_control_task();

// handle/ref to the wifi task to delete it
TaskHandle_t wifi_task_handle = NULL;

// WIFI task prototype
void wifi_task();

// handle/ref to the webserver task to delete it
TaskHandle_t webserver_task_handle = NULL;

//webserver task proto
void webserver_task();

// minmax prototype
int max_int(int x, int y);
int min_int(int x, int y);

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

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

// init wifi proto
void wifi_init_softap();

// handle wifi connections proto
static esp_err_t event_handler(void *ctx, system_event_t *event);

// start the webserver proto
httpd_handle_t start_webserver(void);

// Semaphore to protect steering global variable
SemaphoreHandle_t steeringSemaphore = NULL;

// Steering angle global variabled
int steering_angle_global = 0;

// Semaphore to protect throttle global variable
SemaphoreHandle_t throttleSemaphore = NULL;

//throttle global variable
int throttle_global = 0;

void app_main(){
    printf("hi\n");

    // initialize steering semaphore
    steeringSemaphore = xSemaphoreCreateMutex();

    //initialize throttle semaphore
    throttleSemaphore = xSemaphoreCreateMutex();

    //create the task to blink the led
    xTaskCreate(
        &motor_control_task, //memory address
        "motor_control_task", //name of the task
        2048, //bits of the stack
        NULL, //parameter to pass (empty)
        4, //priority
        motor_control_task_handle //handle/ref to the motor control task to delete it
        );
    
    //create the task for WIFI
     xTaskCreate(
        &wifi_task, //memory address
        "wifi_task", //name of the task
        4096, //bits of the stack
        NULL, //parameter to pass (empty)
        5, //priority
        wifi_task_handle //handle/ref to the wifi task to delete it
        );
}


//blink the led 30times
void motor_control_task(){

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
    int T_on = 0;
    double duty_cycle = 0;
    int duty_cycle_int = 0;

    int throttle = 0;
    double DC_duty_cycle = 0;
    int DC_duty_cycle_int = 0;

    while(1){

        // Calculate and output Servo motor PWM
        //calculate the servo angle from triangle wave
        if(xSemaphoreTake(steeringSemaphore, portMAX_DELAY)){
            servo_angle = steering_angle_global;
            xSemaphoreGive(steeringSemaphore);
        }

        
        printf("Servo angle: %d\n", servo_angle);

        //calculate the T_on in microseconds
        T_on = angle_to_us(servo_angle);
        // printf("T_on: %d\n", T_on);

        //calculate the duty cycle
        duty_cycle = duty_cycle_from_T_on_calc(T_on, STEERING_SERVO_FREQ);
        // printf("Servo duty double: %lf\n", duty_cycle);

        //convert double duty cycle to int duty cycle
        duty_cycle_int = duty_cycle_int_calc(duty_cycle, STEERING_DUTY_RESOLUTION);
        // printf("Servo duty int: %d\n", duty_cycle_int);

        //pass to pin
        set_duty(duty_cycle_int, steering_PWM_config);


        // Calculate and output DC motor PWM
        //calculate throttle from triangle wave
        
        if(xSemaphoreTake(throttleSemaphore, portMAX_DELAY)){
            throttle = throttle_global;
            xSemaphoreGive(throttleSemaphore);
        }

        printf("Throttle: %d\n", throttle);

        //calculate duty cycle from throttle
        DC_duty_cycle = throttle / 100.;
        // printf("DC_duty_cycle: %lf\n", DC_duty_cycle);

        DC_duty_cycle_int = duty_cycle_int_calc(DC_duty_cycle, DC_DUTY_RESOLUTION);
        // printf("DC_duty_cycle_int: %d\n", DC_duty_cycle_int);

        //pass to pin
        set_duty(DC_duty_cycle_int, DC_PWM_config);

        vTaskDelay(100 / portTICK_PERIOD_MS);
        i++;
    }
    printf("Delete task\n");
    vTaskDelete(motor_control_task_handle);
}


static const char *TAG = "wifi softAP";

//configure wifi as access point and communicate through http request
void wifi_task(){

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
    // default ip address: 192.168.4.1

    //create the task for webserver after initializing WIFI
    xTaskCreate(
        &webserver_task, //memory address
        "webserver_task", //name of the task
        4096, //bits of the stack
        NULL, //parameter to pass (empty)
        6, //priority
        webserver_task_handle //handle/ref to the web server task to delete it
    );

    printf("WIFI setup successful\n");
    vTaskDelete(wifi_task_handle);
}


//set up webserver and set up handle for http request
void webserver_task(){

    // start the webserver
    start_webserver();

     while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


/* Our URI handler function to be called during POST /steering request */
esp_err_t steering_handler(httpd_req_t *req)
{
    
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    char content[2];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = min_int(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }
    printf("steering post: %d\n", (int)content[0]);
    //protect steering global variable by taking semaphore
    if(xSemaphoreTake(steeringSemaphore, portMAX_DELAY)){
        steering_angle_global = (int)content[0];
        //release semaphore
        xSemaphoreGive(steeringSemaphore);
    }

    /* Send a simple response */
    //const char resp[] = content;
    httpd_resp_send(req, content, strlen(content));
    return ESP_OK;
}


/* URI handler structure for POST /steering */
httpd_uri_t steering_post = {
    .uri      = "/steering",
    .method   = HTTP_POST,
    .handler  = steering_handler,
    .user_ctx = NULL
};

/* Our URI handler function to be called during POST /throttle request */
esp_err_t throttle_handler(httpd_req_t *req)
{
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    char content[2];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = min_int(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    printf("throttle post: %d\n", (int)content[0]);
    //protect throttle global variable by taking semaphore
    if(xSemaphoreTake(throttleSemaphore, portMAX_DELAY)){
        throttle_global = (int)content[0];
        //release semaphore
        xSemaphoreGive(throttleSemaphore);
    }

    /* Send a simple response */
    //const char resp[] = content;
    httpd_resp_send(req, content, strlen(content));
    return ESP_OK;
}


/* URI handler structure for POST /throttle */
httpd_uri_t throttle_post = {
    .uri      = "/throttle",
    .method   = HTTP_POST,
    .handler  = throttle_handler,
    .user_ctx = NULL
};

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &steering_post);
        httpd_register_uri_handler(server, &throttle_post);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    default:
        break;
    }
    return ESP_OK;
}


void wifi_init_softap()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}


//1000us <- -90deg, 2000us <- 90deg (linear)
//from servo angle to pwm delay
int angle_to_us(int deg){
    int m = 500 / 90;
    int b = 1500;
    int delay_time = m * deg + b;
    delay_time = min_int(max_int(delay_time,1000), 2000);
    return delay_time;
}


int min_int(int x, int y){
    if(x<y){
        return x;
    }else{
        return y;
    }
}


int max_int(int x, int y){
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