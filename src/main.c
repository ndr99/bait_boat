#include "main.h"


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
        motor_control_task, //task function
        "motor_control_task", //name of the task
        2048, //bits of the stack
        NULL, //parameter to pass (empty)
        4, //priority
        &motor_control_task_handle //handle/ref to the motor control task to delete it
        );
    
    //create the task for WIFI
     xTaskCreate(
        wifi_task, //memory address
        "wifi_task", //name of the task
        4096, //bits of the stack
        NULL, //parameter to pass (empty)
        5, //priority
        &wifi_task_handle //handle/ref to the wifi task to delete it
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

    int motor_servo_T_ON_min = 255;
    int motor_servo_T_ON_max = 2255;
    int motor_servo_phi_min = -90;
    int motor_servo_phi_max = 90;

    while(1){

        // Calculate and output Servo motor PWM
        //calculate the servo angle from triangle wave
        if(xSemaphoreTake(steeringSemaphore, portMAX_DELAY)){
            servo_angle = steering_angle_global;
            xSemaphoreGive(steeringSemaphore);
        }

        
        //printf("Servo angle: %d\n", servo_angle);

        //calculate the T_on in microseconds
        T_on = angle_to_us(servo_angle,
                           motor_servo_T_ON_min,
                           motor_servo_T_ON_max,
                           motor_servo_phi_min,
                           motor_servo_phi_max);
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

        //printf("Throttle: %d\n", throttle);

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
        webserver_task, //memory address
        "webserver_task", //name of the task
        4096, //bits of the stack
        NULL, //parameter to pass (e mpty)
        6, //priority
        &webserver_task_handle //handle/ref to the web server task to delete it
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
    char content[req->content_len];

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
    printf("steering post: %d\n", atoi(content));
    //protect steering global variable by taking semaphore
    if(xSemaphoreTake(steeringSemaphore, portMAX_DELAY)){
        steering_angle_global = atoi(content);
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
    char content[req->content_len];

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

    printf("throttle post: %d\n", atoi(content));
    //protect throttle global variable by taking semaphore
    if(xSemaphoreTake(throttleSemaphore, portMAX_DELAY)){
        throttle_global = atoi(content);
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

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}


void wifi_init_softap()
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    //ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

      ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
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





