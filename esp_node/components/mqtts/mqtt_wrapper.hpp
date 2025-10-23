#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "mqtt_client.h"


typedef void (*MqttMessageCallback)(const char* topic, int topic_len, 
                                     const char* data, int data_len);
                                     
class MqttWrapper {

    public:
        MqttWrapper();
        ~MqttWrapper();
        void mqtt_app_start();
        int send_message(const char* topic, const char* payload);
        int subscribe(const char* topic);

        bool isConnected() const { return _connected; }
        void reconnect();

        void setMessageCallback(MqttMessageCallback callback) {
        _message_callback = callback;
    }


    private:

        friend void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
        esp_mqtt_client_handle_t _client;
        bool _connected;
        char _client_id[13]; // 12 characters + null terminator
        MqttMessageCallback _message_callback;
};

