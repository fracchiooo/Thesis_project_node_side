#include "fft_wrapper.hpp"

static const char* TAG = "FFT";
static TaskHandle_t s_task_handle;

FFT_ultrasonic::FFT_ultrasonic() : _ADC_channel(-1), _ADC_unit(-1), _sampling_freq(0), _num_samples(0), _handle(nullptr), _calibrated(false), _fft_result(nullptr), _cali_handle(nullptr)
{
}

FFT_ultrasonic::~FFT_ultrasonic(){
    if(_handle!=nullptr){
        ESP_ERROR_CHECK(adc_continuous_stop(_handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(_handle));
    }
    if(_cali_handle!=nullptr){
        adc_calibration_deinit(_cali_handle);
    }
    if(_fft_result!=nullptr){
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
    _ADC_channel = ADC_channel;
    _ADC_unit = ADC_unit;
    _sampling_freq = sampling_frequency;
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
    _fft_result = (float*) malloc(2*_num_samples * sizeof(float));
    if(_fft_result == NULL){
        ESP_LOGI(TAG, "unable to allocate heap memory for fft samples");
        return nullptr;
    }

    int array_lenght= 4*_num_samples; // each sample is saved in 4 bytes
  
    ESP_ERROR_CHECK(adc_continuous_start(_handle));

    adc_calibration_init();
    if(!_calibrated){
        printf("calibration for converting raw data to MilliVolts has an error");
        return nullptr;
    }

    esp_err_t ret;
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
                if (chan_num < SOC_ADC_CHANNEL_NUM(unit)) {
                    int t;
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(_cali_handle, data, &t)); //converts raw data to calibrated voltage
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
  
    ESP_ERROR_CHECK(adc_continuous_stop(_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(_handle));
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

    //logica fft, considera _fft_result
    esp_err_t ret;

    //preparing the fft algorithm
    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK){
        printf("Not possible to initialize FFT. Error = %i", ret);
        return -1.0;
    }
    
    // Generate hann window
    float wind[_num_samples];
    dsps_wind_hann_f32(wind, _num_samples);

    float signal_real_normalized_vector[_num_samples];
    memcpy(signal_real_normalized_vector, _fft_result, _num_samples * sizeof(float)); // copies the _num_samples elements in the vector


    signal_real_normalized_vector[0]=signal_real_normalized_vector[1]; // removes the first sample which could bring problems in the analysis phase
    normalize(signal_real_normalized_vector, _num_samples, max_value); // normalizing the signal and centering it in 0
 


    for (int i=0 ; i< _num_samples ; i++)
    {
        _fft_result[i*2 + 0] = signal_real_normalized_vector[i] * wind[i]; // Real part is your signal multiply with window to smooth its corners
        _fft_result[i*2 + 1] = 0; // Imaginary part is 0 (real signal)
    }

    //calculates the FFT (translates the signal from time domain to frequency domain)
    dsps_fft2r_fc32(_fft_result, _num_samples);
    // Bit reverse (the result has the bits as inverted, so invert it to have the result itself)
    dsps_bit_rev_fc32(_fft_result, _num_samples);
    dsps_cplx2reC_fc32(_fft_result, _num_samples); // optimization for real signals

     for (int i = 0 ; i < _num_samples/2 ; i++) {
        _fft_result[i]=sqrtf(pow(_fft_result[i * 2 + 0],2.0f) + pow(_fft_result[i * 2 + 1],2.0f)); // computing the magnitude of each frequency of the signal, using real and imaginary part
     }
    // I've checked for half of the signal, because it is symmetric for real signals


    // now we compute the Z score in order to filter frequencies similar to the noise
    float std_dev;
    float mean;
    std_deviation_and_mean(_fft_result, _num_samples/2, &std_dev, &mean);     
    float threshold = 2.0f;
    int idx=0;
    for(int j=(_num_samples/2)-1; j>=0; j--){
        float dat = _fft_result[j];
        float z= (dat - mean) / std_dev;
        if(z > threshold){
            idx=j;
            break; // we break after finding the higher frequency far away from the noise
        }
    }

    // Show power spectrum in 64x10 window from -60 to 0 dB from 0..N/2 samples
    //printf("Signal x1 in log scale");
    //dsps_view(_fft_result, N/2, 64, 10,  -60, 40, '|');
    printf("Signal x1 in absolute scale");
    dsps_view(_fft_result, _num_samples/2, 64, 10,  0, 2, '|');


    float max_frequency=(idx*_sampling_freq)*1.0f/_num_samples*1.0f; // calculates which frequency belongs to that array slot
    printf("the max frequency of the signal is: %f\n", max_frequency);
    return max_frequency;
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
    if (!calibrated) {
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

    // Apply normalization to each element in the array
    for (int i = 0; i < N; i++) {
        array[i] = scale * array[i] + shift;
    }
}

void FFT_ultrasonic::std_deviation_and_mean(float* data, size_t size, float* std_dev, float* mean){
    float sum = 0.0f;
    float quad_sum = 0.0f;
    for(int i=0; i< size; i++){
        sum += data[i];
        quad_sum += pow(data[i], 2.0f);
    }
    *mean = sum/size;
    float variance = (quad_sum / size) - pow(*mean, 2.0f);
    *std_dev = sqrtf(variance);
}
