#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_dsp.h"
#include <stdlib.h>
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"

#define MAX_SAMPLING_FREQUENCY SOC_ADC_SAMPLE_FREQ_THRES_HIGH



class FFT_ultrasonic {

    public:

        FFT_ultrasonic();
        ~FFT_ultrasonic();
        bool begin(uint8_t ADC_channel, uint8_t ADC_unit, int sampling_frequency, int num_samples);
        void setTimeWindow(int time_window);
        int getMaxFrequencyFFT();


    private:

    uint8_t _ADC_channel;
    uint8_t _ADC_unit;
    int _time_window;
    int _sampling_freq;
    int _num_samples;
    uint32_t* _fft_result;
    adc_continuous_handle_t _handle;
    bool _calibrated;
    adc_cali_handle_t _cali_handle;



};
