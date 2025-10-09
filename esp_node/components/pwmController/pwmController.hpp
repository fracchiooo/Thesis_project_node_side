#pragma once
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class PwmController {
public:
    PwmController(gpio_num_t gpio, ledc_channel_t channel, ledc_timer_t timer, bool inverted);
    ~PwmController();
    void init();
    void setFrequency(uint32_t freq_hz);
    void setDuty(float duty_percent);
    void printStatus();

private:
    gpio_num_t _gpio;
    ledc_channel_t _channel;
    ledc_timer_t _timer;
    uint32_t _frequency;
    ledc_timer_bit_t _resolution;
    uint32_t _max_duty;
    float _current_duty=0.0f;
    bool _inverted;
    
    
};