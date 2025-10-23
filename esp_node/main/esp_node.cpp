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
#include "wifi_wrapper.hpp"
#include "mqtt_wrapper.hpp"
#include "cJSON.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "fft_wrapper.hpp"


static const char* TAG = "MAIN";

typedef struct {
    float frequency;
    float duty_frequency;
    float sensed_frequency;
    float finish_after;
    uint32_t start_time;

} DeviceState_t;

static DeviceState_t device_state = {
    .frequency = 0.0,
    .duty_frequency = 0.0,
    .sensed_frequency = 0.0,
    .finish_after = 0.0,
    .start_time=0
};

static SemaphoreHandle_t state_mutex = NULL;
static esp_timer_handle_t schedule_timer = NULL;
static esp_timer_handle_t expiry_timer = NULL;
//static Actuator_temperature actuator;
//static TaskHandle_t mantain_temperatureTask_handle = NULL;
static Sensor_temperature sensor;
static PwmController* pwm_input_40 = NULL;
static PwmController* pwm_input_20 = NULL;

static PwmController* pwm_burst_20 = NULL;
static PwmController* pwm_burst_40 = NULL;

DeviceState_t get_device_state() {
    DeviceState_t state;
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        state = device_state;
        xSemaphoreGive(state_mutex);
    }
    return state;
}

bool initialize_sntp(void) {
    Wifi_wrapper* wifi = Wifi_wrapper::getWifiInstance();
    if (!wifi->isConnected()) {
        ESP_LOGE(TAG, "WiFi not connected, cannot sync time");
        return false;
    }
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    int retry = 0;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < 20) {
        ESP_LOGI(TAG, "Waiting for time sync... (%d/20)", retry);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (retry < 20) {
        time_t now;
        time(&now);
        ESP_LOGI(TAG, "Time synchronized: %s", ctime(&now));
        return true;
    } else {
        ESP_LOGW(TAG, "Time sync timeout");
        return false;
    }
}

/*
void mantain_temperature(void* temperature_target){
    //histeresys cycle
    const float HYSTERESIS = 0.5;  // isteresi di ±0.5°C
    const float TOLERANCE = 0.2;
    const unsigned long DELAY_MS = 5000;

    float* temp = (float*) temperature_target;
    float temp_target = *temp;

    while (true) {
        float currentTemp = sensor.readTemperature(0);
        if(currentTemp==-127.0){
            printf("error in reading temperature in control function");
            actuator.stop_cool();
            actuator.stop_warm();
            mantain_temperatureTask_handle=NULL;
            return;
        }
        float error = temp_target - currentTemp;
        
        if (abs(error) < TOLERANCE) {
            actuator.stop_cool();
            actuator.stop_warm();
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
        
        else if (error > HYSTERESIS) {
            actuator.init_warm(100.0);
        } 
        else if (error < -HYSTERESIS) {
            actuator.init_cool(100.0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    
    }
}*/

void scheduled_action(void* arg) {
    ESP_LOGI(TAG, "Timer triggered! Executing scheduled action...");
    DeviceState_t state = get_device_state();
    // TODO start irradiation
    /*float temp = state.temperature;
    xTaskCreate(mantain_temperature, "mantain_temperatureTask", 4096, (void*)&temp, 2, &mantain_temperatureTask_handle);*/

    if(state.frequency==40.0){
        pwm_input_20->setDuty(0.0f);
        pwm_burst_20->setDuty(0.0f);

        pwm_input_40->setFrequency(39300);
        pwm_input_40->setDuty(50.0f);
        pwm_burst_40->setDuty(state.duty_frequency);

    } else if(state.frequency==20.0){
        pwm_burst_40->setDuty(0.0f);
        pwm_input_40->setDuty(0.0f);

        pwm_input_20->setFrequency(21700);
        pwm_input_20->setDuty(50.0f);
        pwm_burst_20->setDuty(state.duty_frequency);

    } else {
        pwm_input_20->setDuty(0.0f);    
        pwm_input_40->setDuty(0.0f);
        pwm_burst_20->setDuty(0.0f);
        pwm_burst_40->setDuty(0.0f);

    }
}

void expiry_action(void* arg) {
    ESP_LOGI(TAG, "Expiry timer triggered! Schedule has expired!");

    /*if (mantain_temperatureTask_handle != NULL) {
        vTaskDelete(mantain_temperatureTask_handle);
        mantain_temperatureTask_handle = NULL;
    }
    actuator.stop_warm();
    actuator.stop_cool();*/

    pwm_input_20->setDuty(0.0f);
    pwm_input_40->setDuty(0.0f);

    pwm_burst_20->setDuty(0.0f);
    pwm_burst_40->setDuty(0.0f);

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
        expiry_action(NULL);
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
        cJSON *finish_after = cJSON_GetObjectItem(json, "finish_after");
        cJSON *startTime = cJSON_GetObjectItem(json, "startTime");

        if(xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE){

            if (cJSON_IsNumber(frequency) ) {
                device_state.frequency = (float) frequency->valuedouble;
            }
            if (cJSON_IsNumber(duty_frequency)) {
                device_state.duty_frequency = (float) duty_frequency->valuedouble;
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
            xSemaphoreGive(state_mutex);
            //triggers the time clock for start and stop the actuators
            schedule_timer_command(device_state.start_time, device_state.finish_after);
            ESP_LOGI(TAG, "status updated and timer started");
        }

        cJSON_Delete(json);
    } else {
        ESP_LOGW(TAG, "Failed to parse JSON");
    }
}





char* encodePayload(float current_temperature, float current_frequency) {
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
        "\"currentSensedFrequency\":%.2f,"
        "\"deviceEnvRequests\":{"
            "\"frequency\":%.2f,"
            "\"duty_frequency\":%.2f,"
            "\"finishAfter\":%.2f,"
            "\"startTime\":\"%s\""
        "}"
        "}",
        timestamp,
        current_temperature,
        current_frequency,
        curr_state.frequency,
        curr_state.duty_frequency,
        curr_state.finish_after,
        iso_date
    );

    return payload;
}


void check_connection(Wifi_wrapper* wifi, MqttWrapper* mqtt, PwmController* pwm_led){

    const int max_backoff = 17;
    int backoff=0;

    while (!wifi->isConnected()) {
        pwm_led->setDuty(0.0f);
        wifi->reconnect();
        vTaskDelay(pdMS_TO_TICKS(pow(2, backoff) * 250));
        if (backoff < max_backoff) backoff++;
    }

    backoff=0;

    while (!mqtt->isConnected()) {
        pwm_led->setDuty(0.0f);
        mqtt->reconnect();
        vTaskDelay(pdMS_TO_TICKS(pow(2, backoff) * 250));
        if (backoff < max_backoff) backoff++;
    }

    pwm_led->setDuty(20.0f);
}



extern "C" void app_main(void)
{
    state_mutex = xSemaphoreCreateMutex();

    // Inizializza controller PWM
    pwm_input_40 = new PwmController(GPIO_NUM_2, LEDC_CHANNEL_0, LEDC_TIMER_0, false);
    pwm_input_40->init();

    pwm_input_20 = new PwmController(GPIO_NUM_3, LEDC_CHANNEL_3, LEDC_TIMER_3, false);
    pwm_input_20->init();
   
    pwm_burst_20 = new PwmController(GPIO_NUM_4, LEDC_CHANNEL_1, LEDC_TIMER_1, false);
    pwm_burst_20->init();
    pwm_burst_40 = new PwmController(GPIO_NUM_1, LEDC_CHANNEL_2, LEDC_TIMER_1, false);
    pwm_burst_40->init();

    pwm_burst_40->setFrequency(150);
    pwm_burst_20->setFrequency(150);


    //actuator.begin(GPIO_NUM_6, GPIO_NUM_5, LEDC_CHANNEL_2, LEDC_TIMER_2);
    sensor.begin();
    int new_slots[MAX_SENSORS];
    int num = sensor.scan(new_slots);
    ESP_LOGI(TAG, "Inizializzati %d sensori", num);

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
    
    if(!initialize_sntp()){
        return;
    }




    FFT_ultrasonic fft;
    fft.begin(ADC_CHANNEL_6, ADC_UNIT_1, 90000, 1024); // sampling at 90 khz in order to sample signals till 45 khz, taking 900 samples, so sampling with steps of 100 hz
    // the sampling rate should be always <= of MAX_SAMPLING_FREQUENCY

    PwmController* pwm_led = new PwmController(GPIO_NUM_35, LEDC_CHANNEL_3, LEDC_TIMER_1, false);
    pwm_led->init();
    pwm_led->setDuty(20.0f);

    while(1){
        check_connection(wifi, &mqtt, pwm_led);
        sensor.scan(new_slots);
        float curr_temp = sensor.readTemperature(0);

        fft.read_and_get_data_fixed_samples();
        int max_freq = fft.getMaxFrequencyFFT();

        char* payload = encodePayload(curr_temp, max_freq);
        mqtt.send_message(nullptr, payload);
        delete[] payload;
        ESP_LOGI(TAG, "MQTT message sent!");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    delete pwm_input_40;
    delete pwm_input_20;
    delete pwm_burst_20;
    delete pwm_burst_40;
    delete pwm_led;
    //vTaskDelete(NULL);


    
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





}
