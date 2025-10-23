#include "mqtt_wrapper.hpp"

static const char *TAG = "mqtts";

extern const uint8_t emqxsl_ca_crt_start[] asm("_binary_emqxsl_ca_crt_start");
extern const uint8_t emqxsl_ca_crt_end[]   asm("_binary_emqxsl_ca_crt_end");


MqttWrapper::MqttWrapper() : _client(nullptr), _connected(false), _message_callback(nullptr)
{
    uint8_t mac_buf[6];
    esp_read_mac(mac_buf, ESP_MAC_WIFI_STA);
    snprintf(_client_id, sizeof(_client_id), "%02X%02X%02X%02X%02X%02X",
             mac_buf[0], mac_buf[1], mac_buf[2], mac_buf[3], mac_buf[4], mac_buf[5]);
    
    ESP_LOGI(TAG, "MqttWrapper created with client_id: %s", _client_id);
}

MqttWrapper::~MqttWrapper()
{
    ESP_LOGI(TAG, "MqttWrapper destructor called");
    
    if (_connected && _client) {
        ESP_LOGI(TAG, "Disconnecting MQTT client...");
        esp_mqtt_client_disconnect(_client);
        _connected = false;
    }
    
    if (_client) {
        ESP_LOGI(TAG, "Stopping MQTT client...");
        esp_mqtt_client_stop(_client);
    }
    
    if (_client) {
        ESP_LOGI(TAG, "Destroying MQTT client...");
        esp_mqtt_client_destroy(_client);
        _client = nullptr;
    }
    
    ESP_LOGI(TAG, "MQTT cleanup completed");
}



void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    esp_mqtt_client_handle_t client = event->client;
    MqttWrapper* instance = static_cast<MqttWrapper*>(handler_args);
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        instance->_connected = true;
        char topic[64];
        snprintf(topic, sizeof(topic), "%s/downlink", instance->_client_id);
        msg_id = esp_mqtt_client_subscribe(client, topic, 1);
        ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic, msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        instance->_connected = false;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        if (instance->_message_callback != nullptr) {
            instance->_message_callback(event->topic, event->topic_len,
                                       event->data, event->data_len);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            instance->_connected = false;
        } else {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

int MqttWrapper::subscribe(const char* topic){
    int msg_id = esp_mqtt_client_subscribe(_client, topic, 1);
    ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic, msg_id);
    return msg_id;
}


int MqttWrapper::send_message(const char* topic, const char* payload) {
    if(!_connected) {
        ESP_LOGW(TAG, "Client not connected, message not sent");
        return -1;
    }
    
    if(payload == nullptr) {
        ESP_LOGW(TAG, "Payload is null, message not sent");
        return -1;
    }
    
    char default_topic[64];
    const char* actual_topic = topic;
    
    if(topic == nullptr) {
        snprintf(default_topic, sizeof(default_topic), "%s/uplink", _client_id);
        actual_topic = default_topic;
    }
    
    int msg_id = esp_mqtt_client_enqueue(_client, actual_topic, payload, 0, 1, 0, true);
    ESP_LOGI(TAG, "Enqueued to %s, msg_id=%d", actual_topic, msg_id);
    return msg_id;
}



void MqttWrapper::reconnect()
{
    if (_connected) {
        ESP_LOGI(TAG, "Already connected");
        return;
    }
    
    if (_client == nullptr) {
        ESP_LOGW(TAG, "Client not initialized");
        return;
    }

    esp_err_t err = esp_mqtt_client_reconnect(_client);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Reconnection initiated successfully");
        _connected = true;
    }
}








void MqttWrapper::mqtt_app_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = CONFIG_BROKER_URI,
            },
            .verification = {
                .certificate = (const char *)emqxsl_ca_crt_start,
            },
        },
        .credentials = {
            .username = CONFIG_BROKER_USERNAME,
            .client_id = _client_id,
            .authentication = {
                .password = CONFIG_BROKER_PASSWORD,
            },
        },
    };



    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    _client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(_client, MQTT_EVENT_ANY, mqtt_event_handler, this);
    esp_mqtt_client_start(_client);
}