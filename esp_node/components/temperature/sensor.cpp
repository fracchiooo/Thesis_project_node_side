#include "sensor.hpp"
#include <string.h>

static const char* TAG = "Temperature sensor";

// Comandi DS18B20
#define CMD_CONVERT_T       0x44
#define CMD_READ_SCRATCHPAD 0xBE
#define CMD_MATCH_ROM       0x55
#define CMD_SKIP_ROM        0xCC

Sensor_temperature::Sensor_temperature():
    _bus(nullptr),
    _device_count(0)
{
    for (int i = 0; i < MAX_SENSORS; i++) {
        _device_active[i] = false;
        _addresses[i] = 0;
    }
}

Sensor_temperature::~Sensor_temperature() {
    if (_bus) {
        onewire_bus_del(_bus);
    }
}

void Sensor_temperature::begin() {
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = ONEWIRE_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };
    
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &_bus));
    ESP_LOGI(TAG, "Bus inizializzato su GPIO %d", ONEWIRE_GPIO);
}

void Sensor_temperature::reinitBus() {
    ESP_LOGW(TAG, "Reinizializzazione bus...");
    
    if (_bus) {
        onewire_bus_del(_bus);
        _bus = nullptr;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = ONEWIRE_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };
    
    esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &_bus);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Bus reinizializzato");
    } else {
        ESP_LOGE(TAG, "Errore reinizializzazione bus");
    }
}

int Sensor_temperature::findFreeSlot() {
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (!_device_active[i]) {
            return i;
        }
    }
    return -1;
}

int Sensor_temperature::findDeviceByAddress(uint64_t address) {
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (_device_active[i] && _addresses[i] == address) {
            return i;
        }
    }
    return -1;
}

int Sensor_temperature::scan(int* new_slots) {
    onewire_device_iter_handle_t iter = nullptr;
    onewire_device_t device;
    esp_err_t search_result;
    
    search_result = onewire_new_device_iter(_bus, &iter);
    if (search_result != ESP_OK) {
        ESP_LOGE(TAG, "Errore creazione iteratore");
        reinitBus();
        return 0;
    }
    
    int new_count = 0;
    int error_count = 0;
    const int max_errors = 3;
    
    do {
        search_result = onewire_device_iter_get_next(iter, &device);
        
        if (search_result == ESP_OK) {
            error_count = 0;
            uint64_t addr = device.address;
            
            if (findDeviceByAddress(addr) == -1) {
                int slot = findFreeSlot();
                if (slot != -1) {
                    _addresses[slot] = addr;
                    _device_active[slot] = true;
                    _device_count++;
                    
                    new_slots[new_count] = slot;
                    new_count++;
                    
                    ESP_LOGI(TAG, "Slot %d: %016llX", slot, addr);
                }
            }
        } else if (search_result != ESP_ERR_NOT_FOUND) {
            error_count++;
            if (error_count >= max_errors) {
                ESP_LOGW(TAG, "Troppi errori di lettura, reinizializzo bus");
                onewire_del_device_iter(iter);
                reinitBus();
                return new_count;
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    
    onewire_del_device_iter(iter);
    ESP_LOGI(TAG, "Trovati %d nuovi (totale %d)", new_count, _device_count);
    return new_count;
}

void Sensor_temperature::removeDevice(int index) {
    if (index < 0 || index >= MAX_SENSORS || !_device_active[index]) {
        return;
    }
    
    _device_active[index] = false;
    _addresses[index] = 0;
    _device_count--;
    ESP_LOGI(TAG, "Slot %d rimosso", index);
}

float Sensor_temperature::readDS18B20(uint64_t address) {
    // Reset bus
    if (onewire_bus_reset(_bus) != ESP_OK) {
        return -127.0;
    }
    
    // MATCH ROM + indirizzo
    uint8_t tx_buf[9] = {CMD_MATCH_ROM};
    memcpy(&tx_buf[1], &address, 8);
    onewire_bus_write_bytes(_bus, tx_buf, 9);
    
    // CONVERT T
    uint8_t cmd = CMD_CONVERT_T;
    onewire_bus_write_bytes(_bus, &cmd, 1);
    
    // Attendi conversione
    vTaskDelay(pdMS_TO_TICKS(800));
    
    // Reset
    onewire_bus_reset(_bus);
    
    // MATCH ROM di nuovo
    onewire_bus_write_bytes(_bus, tx_buf, 9);
    
    // READ SCRATCHPAD
    cmd = CMD_READ_SCRATCHPAD;
    onewire_bus_write_bytes(_bus, &cmd, 1);
    
    // Leggi 9 byte
    uint8_t data[9];
    if (onewire_bus_read_bytes(_bus, data, 9) != ESP_OK) {
        return -127.0;
    }
    
    // Calcola temperatura
    int16_t raw = (data[1] << 8) | data[0];
    return (float)raw / 16.0;
}

float Sensor_temperature::readTemperature(int index) {
    if (index < 0 || index >= MAX_SENSORS) {
        ESP_LOGE(TAG, "Indice non valido: %d", index);
        return -127.0;
    }
    
    if (!_device_active[index]) {
        ESP_LOGW(TAG, "Slot %d vuoto", index);
        return -127.0;
    }
    
    float temp = readDS18B20(_addresses[index]);
    
    if (temp != -127.0) {
        ESP_LOGI(TAG, "Slot %d: %.2f °C", index, temp);
    }
    
    return temp;
}