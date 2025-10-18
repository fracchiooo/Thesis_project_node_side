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
#include <cstring>
#include <cmath>
#define MAX_SAMPLING_FREQUENCY SOC_ADC_SAMPLE_FREQ_THRES_HIGH



class FFT_ultrasonic {

    public:

        FFT_ultrasonic();
        ~FFT_ultrasonic();
        void begin(uint8_t ADC_channel, uint8_t ADC_unit, uint32_t sampling_frequency, int num_samples);
        int getMaxFrequencyFFT();
        float* read_and_get_data_fixed_samples();


    private:

        uint32_t get_max_cali_value(adc_cali_handle_t calibration);
        void normalize(float array[], int N, uint32_t max_val);
        void std_deviation_and_mean(float* data, size_t size, float* std_dev, float* mean);
        void adc_calibration_init();
        void adc_calibration_deinit(adc_cali_handle_t handle);


        uint8_t _ADC_channel;
        uint8_t _ADC_unit;
        uint32_t _sampling_freq;
        int _num_samples;
        adc_continuous_handle_t _handle;
        bool _calibrated;
        float* _fft_result;
        adc_cali_handle_t _cali_handle;
};
