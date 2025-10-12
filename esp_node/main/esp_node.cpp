#include <stdio.h>
#include <stdio.h>
#include <stdint.h>
#include "pwmController.hpp"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include "driver/gpio.h"
#include <stdexcept>
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_random.h" 
#include "nvs_flash.h"
#include "sensor.hpp"
#include "actuator.hpp"
#include "wifi_wrapper.hpp"
#include "mqtt_wrapper.hpp"
#include "cJSON.h"
#include "esp_timer.h"
#include "esp_sntp.h"


static const char* TAG = "MAIN";

typedef struct {
    float frequency;
    float duty_frequency;
    float temperature;
    float finish_after;
    uint32_t start_time;

} DeviceState_t;

static DeviceState_t device_state = {
    .frequency = 0.0,
    .duty_frequency = 0.0,
    .temperature = -127.0,
    .finish_after = 0.0,
    .start_time=0
};

static SemaphoreHandle_t state_mutex = NULL;
static esp_timer_handle_t schedule_timer = NULL;
static esp_timer_handle_t expiry_timer = NULL;


typedef struct {
    PwmController pwm_input;
    PwmController pwm_burst;
} PWMchannels;

void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    
    // Aspetta sincronizzazione (max 10 secondi)
    int retry = 0;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < 10) {
        ESP_LOGI(TAG, "Waiting for time sync... (%d/10)", retry);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (retry < 10) {
        time_t now;
        time(&now);
        ESP_LOGI(TAG, "Time synchronized: %s", ctime(&now));
    } else {
        ESP_LOGW(TAG, "Time sync timeout");
    }
}

void scheduled_action(void* arg) {
    ESP_LOGI(TAG, "⏰ Timer triggered! Executing scheduled action...");
}

void expiry_action(void* arg) {
    ESP_LOGI(TAG, "⏳ Expiry timer triggered! Schedule has expired!");
}

void schedule_timer_command(uint32_t start_timestamp, float expiry_hours) {

    if (schedule_timer != NULL) {
        esp_timer_stop(schedule_timer);
        esp_timer_delete(schedule_timer);
        schedule_timer = NULL;
        ESP_LOGI(TAG, "Previous schedule timer deleted");
    }
    
    if (expiry_timer != NULL) {
        esp_timer_stop(expiry_timer);
        esp_timer_delete(expiry_timer);
        expiry_timer = NULL;
        ESP_LOGI(TAG, "Previous expiry timer deleted");
    }
    

    time_t now = time(NULL);
    int64_t start_delay_seconds = (int64_t)start_timestamp - (int64_t)now;
    

    if (start_timestamp == 0 || start_delay_seconds <= 0) {
        ESP_LOGI(TAG, "Start timestamp is 0 or past, executing immediately");
        scheduled_action(NULL);
        start_delay_seconds = 0;
    } else {

        int64_t start_delay_us = start_delay_seconds * 1000000LL;
        
        esp_timer_create_args_t timer_args = {
            .callback = scheduled_action,
            .arg = NULL,
            .name = "schedule_timer"
        };
        
        esp_timer_create(&timer_args, &schedule_timer);
        esp_timer_start_once(schedule_timer, start_delay_us);
        
        ESP_LOGI(TAG, "Schedule timer set for %lld seconds from now", start_delay_seconds);
    }
    
    // 4. Calcola e avvia expiry timer
    int64_t expiry_seconds = (int64_t)(expiry_hours * 3600.0f);
    int64_t total_expiry_seconds = start_delay_seconds + expiry_seconds;
    
    if (total_expiry_seconds <= 0) {
        ESP_LOGI(TAG, "Expiry is in the past, triggering immediately");
        expiry_action(NULL);
    } else {
        int64_t expiry_delay_us = total_expiry_seconds * 1000000LL;
        
        esp_timer_create_args_t expiry_args = {
            .callback = expiry_action,
            .arg = NULL,
            .name = "expiry_timer"
        };
        
        esp_timer_create(&expiry_args, &expiry_timer);
        esp_timer_start_once(expiry_timer, expiry_delay_us);
        
        ESP_LOGI(TAG, "Expiry timer set for %.2f hours (total %lld seconds from now)", 
                 expiry_hours, total_expiry_seconds);
    }
}


void onMqttMessage(const char* topic, int topic_len, const char* data, int data_len) {
    ESP_LOGI(TAG, "=== Message received in MAIN ===");
    ESP_LOGI(TAG, "Topic: %.*s", topic_len, topic);
    ESP_LOGI(TAG, "Data: %.*s", data_len, data);
    char payload[512];
    int copy_len = (data_len < sizeof(payload) - 1) ? data_len : sizeof(payload) - 1;
    memcpy(payload, data, copy_len);
    payload[copy_len] = '\0';
    
    cJSON *json = cJSON_Parse(payload);
    if (json != NULL) {
        cJSON *frequency = cJSON_GetObjectItem(json, "frequency");
        cJSON *duty_frequency = cJSON_GetObjectItem(json, "duty_frequency");
        cJSON *temperature = cJSON_GetObjectItem(json, "temperature");
        cJSON *finish_after = cJSON_GetObjectItem(json, "finish_after");
        cJSON *startTime = cJSON_GetObjectItem(json, "startTime");

        if(xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE){

            if (cJSON_IsNumber(frequency) ) {
                device_state.frequency = (float) frequency->valuedouble;
            }
            if (cJSON_IsNumber(duty_frequency)) {
                device_state.duty_frequency = (float) duty_frequency->valuedouble;
                
            }
            if (cJSON_IsNumber(temperature)) {
                device_state.temperature = (float) temperature->valuedouble;
            }
            if (cJSON_IsNumber(finish_after)) {
                device_state.finish_after = (float) finish_after->valuedouble;
            }
            if (cJSON_IsNull(startTime) || cJSON_IsString(startTime)) {
                if(cJSON_IsNull(startTime)){
                    time_t now = time(NULL);
                    struct tm timeinfo;
                    gmtime_r(&now, &timeinfo);
                    device_state.start_time = mktime(&timeinfo);
                } else {
                    const char* date_str = startTime->valuestring;
                    struct tm tm;
                    memset(&tm, 0, sizeof(tm));
                    sscanf(date_str, "%d-%d-%dT%d:%d:%d",
                        &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
                        &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
                    tm.tm_year -= 1900;
                    tm.tm_mon -= 1;
                    
                    device_state.start_time = mktime(&tm);
                    ESP_LOGI(TAG, "Parsed timestamp: %lu", device_state.start_time);

                }
            }
            //triggers the time clock for start and stop the actuators
            schedule_timer_command(device_state.start_time, device_state.finish_after);
            ESP_LOGI(TAG, "status updated and timer started");
        }

        xSemaphoreGive(state_mutex);
        cJSON_Delete(json);
    } else {
        ESP_LOGW(TAG, "Failed to parse JSON");
    }
}

DeviceState_t get_device_state() {
    DeviceState_t state;
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        state = device_state;
        xSemaphoreGive(state_mutex);
    }
    return state;
}


bool isParsableInt(const std::string &s) {
    try {
        size_t pos;
        std::stoi(s, &pos);           // tenta la conversione
        return pos == s.size();       // controlla che tutta la stringa sia stata consumata
    } catch (std::invalid_argument& exception) {
        return false;                 // eccezione → non parsabile
    }
}


char* encodePayload(float current_temperature) {
    char* payload = new char[512];
        
    // get current time
    time_t now = time(NULL);
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);

    DeviceState_t curr_state = get_device_state();

    time_t start_time_enc = curr_state.start_time;
    gmtime_r(&start_time_enc, &timeinfo);
    char iso_date[32];
    strftime(iso_date, sizeof(iso_date), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);
    
    snprintf(payload, 512,
        "{"
        "\"lastUpdate\":\"%s\","
        "\"currentTemperature\":%.2f,"
        "\"deviceEnvRequests\":{"
            "\"temperature\":%.2f,"
            "\"frequency\":%.2f,"
            "\"duty_frequency\":%.2f,"
            "\"finishAfter\":%.2f,"
            "\"startTime\":\"%s\""
        "}"
        "}",
        timestamp,
        current_temperature,
        curr_state.temperature,
        curr_state.frequency,
        curr_state.duty_frequency,
        curr_state.finish_after,
        iso_date
    );

    return payload;
}



// TODO cambiare con uso protocollo wifi + mqtt

/*
void loraTask(void* param) {

    esp_task_wdt_config_t config= {
        .timeout_ms = 30000,
        .idle_core_mask = 0,
        .trigger_panic = false
    };

    esp_task_wdt_init(&config); // 10s timeout
    esp_task_wdt_add(NULL);      // add current task
    PWMchannels* pwm_channels = (PWMchannels*) param;
    PwmController pwm = (*pwm_channels).pwm_input;
    PwmController pwm_burst = (*pwm_channels).pwm_burst;

    init_lora();


    while(true) {
    char message[256];
    if (receive_lora_data(message)) {
        printf("Received message: %s\n", message);
        int value = -1;

        bool burst_freq = false;

        if(strncmp(message, "_", 1) == 0){
            burst_freq = true;
            // Rimuovi il carattere di underscore per la parsificazione
            std::string msg_str(message);
            msg_str = msg_str.substr(1); // Rimuovi il primo carattere
            strncpy(message, msg_str.c_str(), sizeof(message));
            message[sizeof(message) - 1] = '\0'; // Assicurati che sia null-terminated
            printf("Burst mode activated");
        }

        if(isParsableInt(message)){
            value = std::stoi(message);
            printf("Parsed integer: %d\n", value);
            if(!burst_freq && value >= 10 && value <= 60000){
                pwm.setFrequency(value);
                pwm.setDuty(50.0f);
            } else if(burst_freq && value >= 0 and value <= 100){
                pwm_burst.setFrequency(5000);
                pwm_burst.setDuty((float) value);
            } else{
                printf("Value out of range. Shutting down the PWMs.\n");
                pwm.setDuty(0.0f);
                pwm_burst.setDuty(0.0f);
            }

        } else {
            printf("Message is not a valid integer.\n");
        }
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay di 100ms invece di 10ms
    }
}*/


extern "C" void app_main(void)
{
    state_mutex = xSemaphoreCreateMutex();

    // Inizializza controller PWM
    PwmController pwm(GPIO_NUM_2, LEDC_CHANNEL_0, LEDC_TIMER_0, false);
    pwm.init();
   
    PwmController pwm_burst(GPIO_NUM_20, LEDC_CHANNEL_1, LEDC_TIMER_1, false);
    pwm_burst.init();

    PWMchannels pwm_channels = {pwm, pwm_burst};


    // Inizializza WiFi

    Wifi_wrapper* wifi = Wifi_wrapper::getWifiInstance();

    wifi->wifi_init_sta();


    int retry = 0;
    while (!wifi->isConnected() && retry < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retry++;
    }
    
    if (!wifi->isConnected()) {
        ESP_LOGE(TAG, "WIFI connection failed!");
        return;
    }
    
    ESP_LOGI(TAG, "WIFI connected!");
    
    //wifi->destroyInstance();


    // Inizializza MQTT
    MqttWrapper mqtt;
    mqtt.setMessageCallback(onMqttMessage);
    mqtt.mqtt_app_start();

    retry = 0;
    while (!mqtt.isConnected() && retry < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retry++;
    }
    
    if (!mqtt.isConnected()) {
        ESP_LOGE(TAG, "MQTT connection failed!");
        return;
    }
    
    ESP_LOGI(TAG, "MQTT connected! System running in background...");
    initialize_sntp();


    while(1){
        char* payload = encodePayload(777.0f);
        mqtt.send_message(nullptr, payload);
        delete[] payload;

        ESP_LOGI(TAG, "MQTT message sent!");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    //vTaskDelete(NULL);

    /*
    if (schedule_timer != NULL) {
        esp_timer_stop(schedule_timer);
        esp_timer_delete(schedule_timer);
        schedule_timer = NULL;
    }
    if (expiry_timer != NULL) {
        esp_timer_stop(expiry_timer);
        esp_timer_delete(expiry_timer);
        expiry_timer = NULL;
    }*/


    
    // utilizzo sensore temperatura DS18B20
/*

    Sensor_temperature sensor;
    sensor.begin();
    int new_slots[MAX_SENSORS];
    int num = sensor.scan(new_slots);
    ESP_LOGI(TAG, "Inizializzati %d sensori", num);
    while (1) {
        float temp = sensor.readTemperature(0);
        ESP_LOGI(TAG, "Temp 1 : %.2f °C", temp);
        temp = sensor.readTemperature(1);
        ESP_LOGI(TAG, "Temp 2 : %.2f °C", temp);
        // Riscansiona ogni tanto
        sensor.scan(new_slots);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }*/


/*

    //utilizzo temperature actuator
    Actuator_temperature actuator;
    actuator.begin(GPIO_NUM_7, GPIO_NUM_6, GPIO_NUM_5, LEDC_CHANNEL_2, LEDC_TIMER_2);

    actuator.set_duty(100.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));

    actuator.init_cool();
    actuator.get_status();
    vTaskDelay(pdMS_TO_TICKS(600000));

    actuator.init_warm();
    actuator.get_status();
    vTaskDelay(pdMS_TO_TICKS(10000));


    actuator.set_duty(50.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));


    actuator.set_duty(15.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));

    actuator.stop_warm();

*/

    // cambia con wifi + mqtt
    //xTaskCreate(loraTask, "loraTask", 4096, (void*)&pwm_channels, 2, NULL);



}
