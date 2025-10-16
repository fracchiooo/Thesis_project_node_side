#include "fft_wrapper.hpp"

static const char* TAG = "FFT";
static TaskHandle_t s_task_handle;

FFT_ultrasonic::FFT_ultrasonic() : _ADC_channel(-1), _ADC_unit(-1), _time_window(0), _sampling_freq(0), _num_samples(0), _handle(nullptr), _calibrated(false)
{
}


FFT_ultrasonic::~FFT_ultrasonic(){
    delete[] _fft_result;
    if(_handle!=nullptr){
        ESP_ERROR_CHECK(adc_continuous_stop(_handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(_handle));
    }
    if(_cali_handle!=nullptr){
        adc_calibration_deinit(_cali_handle);
    }

}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}


bool FFT_ultrasonic::begin(uint8_t ADC_channel, uint8_t ADC_unit, int sampling_frequency, int num_samples){

    _fft_result = (uint32_t*) malloc(2*num_samples * 32));
    if(_fft_result == NULL){
        ESP_LOGI(TAG, "unable to allocate heap memory for fft samples");
        return false;
    }
    _ADC_channel = ADC_channel;
    _ADC_unit = ADC_unit;
    _sampling_freq = sampling_frequency;
    _num_samples = num_samples;

    int pool_length = 4*_num_samples; //each sample is saved in 4 bytes

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
        .sample_freq_hz = sampling_frequency,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .adc_pattern=&adc_pattern,
        .pattern_num=1,
    };

    s_task_handle = xTaskGetCurrentTaskHandle();

    ESP_ERROR_CHECK(adc_continuous_config(_handle, &dig_cfg));

     adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

}


uint32_t* read_and_get_data_fixed_samples(){


    int array_lenght= 4*_num_samples; // each sample is saved in 4 bytes
  
    ESP_ERROR_CHECK(adc_continuous_start(_handle));

    bool do_calibration1_chan0 = adc_calibration_init();
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
            uint32_t chan_num = ADC_GET_CHANNEL(p);
            uint32_t data = ADC_GET_DATA(p);
            if (chan_num < SOC_ADC_CHANNEL_NUM(unit)) {
              ESP_ERROR_CHECK(adc_cali_raw_to_voltage(_cali_handle, data, _fft_result[ii])); //converts raw data to calibrated voltage
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
      }
  }
  
  
  ESP_ERROR_CHECK(adc_continuous_stop(_handle));
  ESP_ERROR_CHECK(adc_continuous_deinit(_handle));
  
}  



FFT_ultrasonic::setTimeWindow(int time_window){
    if (_handle==nullptr){
        return;
    }
}


int FFT_ultrasonic::getMaxFrequencyFFT(){

  uint32_t max_value = get_max_cali_value(adc1_cali_chan0_handle);

  if(_fft_result == nullptr){
    ESP_LOGI(TAG, "you should read and sample the signal before try to get its maximum frequency")
    return -1;
  }
  //logica fft, considera _fft_result



}



uint32_t get_max_cali_value(adc_cali_handle_t calibration){

    uint32_t t;
    uint32_t max_raw_value= pow(2, SOC_ADC_DIGI_MAX_BITWIDTH) -1;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration, max_raw_value, &t));
    return t;
}


static adc_calibration_init()
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    _calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
//this is supported by esp32S3
//printf("ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED \n");
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = _ADC_unit,
            .chan = _ADC_channel,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
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

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

#endif