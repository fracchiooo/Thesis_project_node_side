#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pwmController.hpp"


class Actuator_temperature {
private:

    bool _is_cooling;
    bool _is_warming;
    float _duty;
    bool _ready;
    gpio_num_t _in1;
    gpio_num_t _in2;
    gpio_num_t _en;
    PwmController* _pwmContr;
   

public:

    struct status{
        bool is_cooling;
        bool is_warming;
        float duty;
        const char* error;
    };

    Actuator_temperature();
    ~Actuator_temperature();

    void begin(int input_cool, int input_warm, int input_duty,  ledc_channel_t channel_pwm, ledc_timer_t timer);
    Actuator_temperature::status init_warm();
    Actuator_temperature::status stop_warm();
    Actuator_temperature::status init_cool();
    Actuator_temperature::status stop_cool();
    Actuator_temperature::status get_status();
    Actuator_temperature::status set_duty(float duty);

};
