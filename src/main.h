#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include <math.h>
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_wifi_types.h"
#include <string.h>
#include "esp_http_server.h"
#include "esp_netif.h"
#include "servo_control.h"
#include "helper.h"

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

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

// init wifi proto
void wifi_init_softap();

// handle wifi connections proto
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                     int32_t event_id, void* event_data);

// start the webserver proto
httpd_handle_t start_webserver(void);
