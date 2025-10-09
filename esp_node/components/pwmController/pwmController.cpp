#include "pwmController.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include <inttypes.h>
#include "esp_clk_tree.h"
#include <math.h>

static const char* TAG = "PWM";

PwmController::PwmController(gpio_num_t gpio, ledc_channel_t channel, ledc_timer_t timer, bool inverted)
    : _gpio(gpio), _channel(channel), _timer(timer), _frequency(40000), 
      _resolution(LEDC_TIMER_8_BIT), _inverted(inverted) {}

PwmController::~PwmController(){}


void PwmController::init() {
    gpio_set_direction(_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(_gpio, 0);

    // Calcola la risoluzione ottimale per la frequenza desiderata
    uint32_t clock_freq_value;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_RC_FAST, ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT, &clock_freq_value));
    
    uint32_t optimal_resolution = ledc_find_suitable_duty_resolution(clock_freq_value, _frequency);
    _resolution = (ledc_timer_bit_t)optimal_resolution;
    
    ESP_LOGI(TAG, "Risoluzione ottimale calcolata: %ld bit per frequenza %lu Hz", optimal_resolution, _frequency);

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = _resolution,
        .timer_num = _timer,
        .freq_hz = _frequency,
        .clk_cfg = LEDC_USE_RC_FAST_CLK, //~17.8 MHz
        .deconfigure = false,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {
        .gpio_num = _gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = _channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = _timer,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = _inverted
        },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    
    // Calcola _max_duty basandosi sulla risoluzione effettiva
    _max_duty = (1UL << _resolution);
    
    ESP_LOGI(TAG, "PWM inizializzato: GPIO %d, Freq %lu Hz, Risoluzione %d bit (%lu livelli)", 
             _gpio, _frequency, _resolution, _max_duty);
}

void PwmController::setFrequency(uint32_t freq_hz) {
    // Calcola la nuova risoluzione ottimale per la nuova frequenza
    uint32_t clock_freq_value;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_RC_FAST, ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT, &clock_freq_value));
    
    uint32_t new_resolution = ledc_find_suitable_duty_resolution(clock_freq_value, freq_hz);
    
    ESP_LOGI(TAG, "Cambio frequenza: %lu -> %lu Hz, risoluzione: %d -> %ld bit", 
             _frequency, freq_hz, _resolution, new_resolution);
    
    // Riconfigura sempre il timer per evitare inconsistenze
    _resolution = (ledc_timer_bit_t)new_resolution;
    _max_duty = (1UL << _resolution);
    
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = _resolution,
        .timer_num = _timer,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_USE_RC_FAST_CLK,
        .deconfigure = false,
    };
    
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Errore configurazione timer per %lu Hz con %d bit: %s", 
                 freq_hz, _resolution, esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    }
    
    _frequency = freq_hz;
    
    ESP_LOGI(TAG, "Timer riconfigurato: %lu Hz, %d bit (%lu livelli)", 
             freq_hz, _resolution, _max_duty);
    
    // Reimposta il duty cycle attuale per adattarlo alla nuova configurazione
    setDuty(_current_duty);
}

void PwmController::setDuty(float duty_percent) {
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    // Usa la risoluzione effettiva del timer (_resolution e _max_duty sono già aggiornati)
    uint32_t upd_duty = (uint32_t)round(duty_percent * (_max_duty - 1) / 100.0f);
    
    // Assicurati che il duty non superi il massimo possibile
    if (upd_duty >= _max_duty) {
        upd_duty = _max_duty - 1;
    }

    printf("Imposto duty: %lu / %lu (risoluzione: %d bit)\n", upd_duty, _max_duty, _resolution);

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, upd_duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Errore impostazione duty cycle: %s", esp_err_to_name(ret));
    } else {
        // Calcola la percentuale effettiva per il log
        float actual_percent = (float)upd_duty * 100.0f / (_max_duty - 1);
        ESP_LOGI(TAG, "Hardware PWM duty: %.2f%% richiesto -> %.2f%% effettivo (%lu/%lu)",
                 duty_percent, actual_percent, (unsigned long)upd_duty, (unsigned long)_max_duty);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ERRORE CRITICO nel setDuty: %s", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    }

    _current_duty = duty_percent;
}

void PwmController::printStatus() {
    // Verifica la frequenza effettiva del timer
    uint32_t actual_freq = ledc_get_freq(LEDC_LOW_SPEED_MODE, _timer);
    
    ESP_LOGI(TAG, "Frequenza impostata: %lu Hz, effettiva: %lu Hz", 
             (unsigned long)_frequency, (unsigned long)actual_freq);
    ESP_LOGI(TAG, "Duty cycle: %.2f%%", _current_duty);
    ESP_LOGI(TAG, "Risoluzione: %d-bit (%lu livelli)", _resolution, (unsigned long)_max_duty);
    ESP_LOGI(TAG, "Canale invertito: %d", _inverted);
    ESP_LOGI(TAG, "GPIO: %d", _gpio);
    
    // Mostra informazioni aggiuntive per il debug
    uint32_t current_duty_value = ledc_get_duty(LEDC_LOW_SPEED_MODE, _channel);
    float actual_duty_percent = (float)current_duty_value * 100.0f / (_max_duty - 1);
    ESP_LOGI(TAG, "Duty effettivo hardware: %lu/%lu (%.2f%%)", 
             (unsigned long)current_duty_value, (unsigned long)_max_duty, actual_duty_percent);
}