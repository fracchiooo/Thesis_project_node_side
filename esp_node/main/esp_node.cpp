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


static const char* TAG = "MAIN";




typedef struct {
    PwmController pwm_input;
    PwmController pwm_burst;
} PWMchannels;


bool isParsableInt(const std::string &s) {
    try {
        size_t pos;
        std::stoi(s, &pos);           // tenta la conversione
        return pos == s.size();       // controlla che tutta la stringa sia stata consumata
    } catch (std::invalid_argument& exception) {
        return false;                 // eccezione → non parsabile
    }
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

    // Inizializza controller PWM
    PwmController pwm(GPIO_NUM_2, LEDC_CHANNEL_0, LEDC_TIMER_0, false);
    pwm.init();
   
    PwmController pwm_burst(GPIO_NUM_20, LEDC_CHANNEL_1, LEDC_TIMER_1, false);
    pwm_burst.init();

    PWMchannels pwm_channels = {pwm, pwm_burst};



    // Inizializza WiFi

    Wifi_wrapper* wifi = Wifi_wrapper::getWifiInstance();

    wifi->wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(60000));
    wifi->destroyInstance();


    
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
