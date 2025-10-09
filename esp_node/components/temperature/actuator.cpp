#include "actuator.hpp"
#include "esp_log.h"


static const char* TAG = "Temperature actuator";


Actuator_temperature::Actuator_temperature()
{
    _is_cooling = 0; 
    _is_warming = 0;
    _duty=0.0f;
    _ready=0;
    _in1 = (gpio_num_t) 0;
    _in2 = (gpio_num_t) 0;
    _en = (gpio_num_t) 0;
    _pwmContr = nullptr;
}

Actuator_temperature::~Actuator_temperature(){
    stop_warm();
    stop_cool();
    set_duty(0.0f);
    delete _pwmContr;
    _ready=0;
    if(_is_cooling != 0 || _is_warming !=0 || _duty!=0.0f){
        ESP_LOGE(TAG, "Failed to stop actuator during removal");
    }
}

void Actuator_temperature::begin(int input_cool, int input_warm, int input_duty, ledc_channel_t channel_pwm, ledc_timer_t timer){
    _in1= (gpio_num_t) input_cool;
    _in2 = (gpio_num_t) input_warm;
    _en = (gpio_num_t) input_duty; 
    gpio_set_direction(_in1, GPIO_MODE_OUTPUT);
    gpio_set_direction(_in2, GPIO_MODE_OUTPUT);
    _pwmContr = new PwmController(_en, channel_pwm, timer, false);
    _pwmContr->init();
    _pwmContr->setFrequency(25000);
    _pwmContr->setDuty(_duty);
    _ready=1;
    ESP_LOGI(TAG, "Set the actuator connectors succesfully");
}

Actuator_temperature::status Actuator_temperature::init_warm(){
    if(!_ready){
        ESP_LOGE(TAG, "Actuator not ready, try to begin()");
        return {_is_cooling, _is_warming, _duty, "Actuator not ready"};
    }
    status res = get_status();
    if(res.is_cooling){
        ESP_LOGI(TAG, "Stop cooling ...");
        stop_cool();
    }
    gpio_set_level(_in2, 1);
    _is_warming=1;
    ESP_LOGI(TAG, "Started to warm");
    return get_status();
}


Actuator_temperature::status Actuator_temperature::stop_warm(){
    gpio_set_level(_in2, 0);
    _is_warming=0;
    vTaskDelay(pdMS_TO_TICKS(4000)); // waiting for temperature stabilization
    ESP_LOGI(TAG, "Stopped to warm");
    return get_status();
}

Actuator_temperature::status Actuator_temperature::init_cool(){
    if(!_ready){
        ESP_LOGE(TAG, "Actuator not ready, try to begin()");
        return {_is_cooling, _is_warming, _duty, "Actuator not ready"};
    }
    status res = get_status();
    if(res.is_warming){
        ESP_LOGI(TAG, "Stop warming ...");
        stop_warm();
    }
    gpio_set_level(_in1, 1);
    _is_cooling=1;
    ESP_LOGI(TAG, "Started to cool");
    return get_status();
}

Actuator_temperature::status Actuator_temperature::stop_cool(){
    gpio_set_level(_in1, 0);
    _is_cooling=0;
    vTaskDelay(pdMS_TO_TICKS(4000)); // waiting for temperature stabilization
    ESP_LOGI(TAG, "Stopped to cool");
    return get_status();
}

Actuator_temperature::status Actuator_temperature::get_status(){
    return {_is_cooling, _is_warming, _duty, nullptr};
}

Actuator_temperature::status Actuator_temperature::set_duty(float duty){
    _duty=duty;
    _pwmContr->setDuty(_duty);
    ESP_LOGI(TAG, "Set duty cycle with value: %f", _duty);
    return get_status();
}
