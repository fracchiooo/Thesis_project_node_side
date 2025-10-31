#include "fft_wrapper.hpp"

static const char* TAG = "FFT";
static TaskHandle_t s_task_handle;

FFT_ultrasonic::FFT_ultrasonic() : _ADC_channel(-1), _ADC_unit(-1), _sampling_freq(0), _num_samples(0), _handle(nullptr), _calibrated(false), _fft_result(nullptr), _cali_handle(nullptr)
{
}

FFT_ultrasonic::~FFT_ultrasonic(){
    if(_handle != nullptr){
        esp_err_t ret = adc_continuous_stop(_handle);
        if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE){
            ESP_LOGE(TAG, "Error stopping ADC: %s", esp_err_to_name(ret));
        }
        
        ret = adc_continuous_deinit(_handle);
        if(ret != ESP_OK){
            ESP_LOGE(TAG, "Error deiniting ADC: %s", esp_err_to_name(ret));
        }
    }
    
    if(_cali_handle != nullptr){
        adc_calibration_deinit(_cali_handle);
    }
    
    if(_fft_result != nullptr){
        free(_fft_result);
    }
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

void FFT_ultrasonic::begin(uint8_t ADC_channel, uint8_t ADC_unit, uint32_t sampling_frequency, int num_samples){

    if(_handle != nullptr) {
        ESP_LOGE(TAG, "ADC already configured");
        return;
    }

    if(MAX_SAMPLING_FREQUENCY < sampling_frequency){
        ESP_LOGI(TAG, "sampling rate too high, the maximum sampling rate will be used: %d", MAX_SAMPLING_FREQUENCY);
        _sampling_freq = MAX_SAMPLING_FREQUENCY;
    } else {
        _sampling_freq = sampling_frequency;
    }
    _ADC_channel = ADC_channel;
    _ADC_unit = ADC_unit;
    _num_samples = num_samples;

    uint32_t pool_length = 4*_num_samples; //each sample is saved in 4 bytes

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = pool_length,
        .conv_frame_size = pool_length, //reads all the data from ADC in one step
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &_handle));


    adc_digi_pattern_config_t adc_pattern = {
        .atten= ADC_ATTEN_DB_12,
        .channel= _ADC_channel,
        .unit= _ADC_unit,
        .bit_width= SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    adc_continuous_config_t dig_cfg = {
        .pattern_num=1,
        .adc_pattern = &adc_pattern,
        .sample_freq_hz = _sampling_freq,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    s_task_handle = xTaskGetCurrentTaskHandle();

    ESP_ERROR_CHECK(adc_continuous_config(_handle, &dig_cfg));

     adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(_handle, &cbs, NULL));
}

float* FFT_ultrasonic::read_and_get_data_fixed_samples(){

    if(_fft_result == nullptr){
        _fft_result = (float*) malloc(2*_num_samples * sizeof(float));
        if(_fft_result == nullptr){
            ESP_LOGI(TAG, "unable to allocate heap memory for fft samples");
            return nullptr;
        }
    }

    adc_calibration_init();
    if(!_calibrated){
        printf("calibration for converting raw data to MilliVolts has an error");
        return nullptr;
    }


    int array_lenght= 4*_num_samples; // each sample is saved in 4 bytes
  
    esp_err_t ret = adc_continuous_start(_handle);
    if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE){
        ESP_LOGE(TAG, "Failed to start ADC: %s", esp_err_to_name(ret));
        return nullptr;
    }

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
  
    uint32_t ret_num = 0;
    uint8_t result[array_lenght];
    memset(result, 0x00, array_lenght);
    while(1){
        ret = adc_continuous_read(_handle, result, array_lenght, &ret_num, 0);
        if (ret == ESP_OK) {
            int ii=0;
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = p->type2.channel;
                uint32_t data = p->type2.data;
                if (chan_num < SOC_ADC_CHANNEL_NUM(_ADC_unit)) {
                    int t;
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(_cali_handle, data, &t)); //converts raw data to calibrated voltage (4096 discrete levels)
                    _fft_result[ii] = (uint32_t) t;
                } else {
                    printf("error in reading sampled raw data");
                }
                ii++;
            }
        
            vTaskDelay(1);
            break;
        } else if (ret == ESP_ERR_TIMEOUT) {
                vTaskDelay(10);
                continue;
        } else {
            return nullptr;
        }
    }
  
    ret = adc_continuous_stop(_handle);
    if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE){
        ESP_LOGE(TAG, "Failed to stop ADC: %s", esp_err_to_name(ret));
    }
    return _fft_result;
}  

int FFT_ultrasonic::getMaxFrequencyFFT(){

    if(_fft_result == nullptr){
        ESP_LOGI(TAG, "you should read and sample the signal before try to get its maximum frequency");
        return -1.0;
    }

    if(_cali_handle == nullptr){
        ESP_LOGI(TAG, "you should set the calibration before try to get its maximum frequency");
        return -1.0;
    }

    uint32_t max_value = get_max_cali_value(_cali_handle);

    const int NUM_ACQUISITIONS = 5;
    float best_magnitude = 0.0f;
    int best_frequency_bin = 0;
    float best_noise_floor = 0.0f;

    for(int acq = 0; acq < NUM_ACQUISITIONS; acq++) {
        
        float* new_samples = read_and_get_data_fixed_samples();
        if(new_samples == nullptr) {
            continue;
        }

        esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
        if (ret != ESP_OK){
            return -1.0;
        }
        
        float wind[_num_samples];
        dsps_wind_hann_f32(wind, _num_samples);

        float signal_normalized[_num_samples];
        memcpy(signal_normalized, _fft_result, _num_samples * sizeof(float));
        normalize(signal_normalized, _num_samples, max_value);

        for (int i=0; i < _num_samples; i++) {
            _fft_result[i*2 + 0] = signal_normalized[i] * wind[i];
            _fft_result[i*2 + 1] = 0;
        }

        dsps_fft2r_fc32(_fft_result, _num_samples);
        dsps_bit_rev_fc32(_fft_result, _num_samples);
        dsps_cplx2reC_fc32(_fft_result, _num_samples);

        for (int i = 0; i < _num_samples/2; i++) {
            _fft_result[i] = sqrtf(pow(_fft_result[i * 2 + 0], 2.0f) + 
                                   pow(_fft_result[i * 2 + 1], 2.0f));
        }

        const float MIN_FREQ = 300.0f;
        int min_bin = (int)(MIN_FREQ * _num_samples / _sampling_freq);
        
        int max_idx = 0;
        float max_mag = _fft_result[min_bin];
        
        for (int i = min_bin; i < _num_samples/2; i++) {
            if(_fft_result[i] > max_mag) {
                max_mag = _fft_result[i];
                max_idx = i;
            }
        }

        float noise_floor = 0.0f;
        int noise_count = 0;
        for (int i = min_bin; i < _num_samples/2; i++) {
            noise_floor += _fft_result[i];
            noise_count++;
        }
        noise_floor /= noise_count;

        if(max_mag > best_magnitude) {
            best_magnitude = max_mag;
            best_frequency_bin = max_idx;
            best_noise_floor = noise_floor;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    const float MIN_FREQ = 300.0f;
    int min_bin = (int)(MIN_FREQ * _num_samples / _sampling_freq);
    
    float noise_floor = 0.0f;
    int noise_count = 0;
    
    for (int i = min_bin; i < _num_samples/2; i++) {
        noise_floor += _fft_result[i];
        noise_count++;
    }
    noise_floor /= noise_count;

    // Minimum magnitude, it is only 20, because the frequency sensor is not calibrated for the ultrasounds, so their amplitudes are attenuated
    const float MIN_MAGNITUDE = 20.0f;
    
    if (best_magnitude < MIN_MAGNITUDE) {
        ESP_LOGI(TAG, "Sensed a signal too weak (magnitude: %.2f < %.2f mV)", 
                 best_magnitude, MIN_MAGNITUDE);
        return 0;
    }

    // noise threshold
    const float SNR_THRESHOLD = 3.0f; // signal to noise ratio: the signal should be 3x wrt the noise to be considered as valid
    
    if (best_magnitude < best_noise_floor * SNR_THRESHOLD) {
        ESP_LOGI(TAG, "No significant frequency found (magnitude: %.2f, noise floor: %.2f, SNR: %.2f)", 
                 best_magnitude, best_noise_floor, best_magnitude / best_noise_floor);
        return 0;
    }

    float dominant_frequency = (float) best_frequency_bin * _sampling_freq / _num_samples;

    ESP_LOGI(TAG, "Dominant Frequency: %.2f Hz, Magnitude: %.2f mV (attenuated by the sensor), Noise floor: %.2f, SNR: %.2f", 
            dominant_frequency, best_magnitude, best_noise_floor, best_magnitude / best_noise_floor);

    return dominant_frequency;
}

uint32_t FFT_ultrasonic::get_max_cali_value(adc_cali_handle_t calibration){

    int t;
    uint32_t max_raw_value= pow(2, SOC_ADC_DIGI_MAX_BITWIDTH) -1;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration, max_raw_value, &t));
    return (uint32_t)t;
}

void FFT_ultrasonic::adc_calibration_init()
{
    esp_err_t ret = ESP_FAIL;
    _calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
//this is supported by esp32S3
//printf("ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED \n");
    if (!_calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = (adc_unit_t) _ADC_unit,
            .chan = (adc_channel_t) _ADC_channel,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = (adc_bitwidth_t) SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &_cali_handle);
        if (ret == ESP_OK) {
            _calibrated = true;
        }
    }
#endif


#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
//printf("ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED \n");
    if (!_calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = _ADC_unit,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &_cali_handle);
        if (ret == ESP_OK) {
            _calibrated = true;
        }
    }
#endif

}

void FFT_ultrasonic::adc_calibration_deinit(adc_cali_handle_t handle)
{
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
    #endif

    _cali_handle = nullptr;
}

void FFT_ultrasonic::normalize(float array[], int N, uint32_t max_val) {
    // Define the minimum and maximum values in the original range
    float min_value = 0.0;
    float max_value = (float)(max_val*1.0f);

    // Define the minimum and maximum values in the target range
    float target_min = -0.99;
    float target_max = 0.99;

    // Calculate the scaling factor and shift
    float scale = (target_max - target_min) / (max_value - min_value);
    float shift = target_min - scale * min_value; 
    //shift the signal such that values (for example, if 12 bits used for ADC input values) in range [0-4096] will be normalized in [0, 1.98] and shifted in [-0.99, 0,99], so the signal is centered in 0 (no DC component)

    // Apply normalization and DC shift to each element in the array
    for (int i = 0; i < N; i++) {
        array[i] = scale * array[i] + shift;
    }
}

