#include "sensor.hpp"
#include <string.h>

static const char* TAG = "Temperature sensor";

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
    if(_new_slots != nullptr){
        free(_new_slots);
    }
}

void Sensor_temperature::begin(gpio_num_t input) {
    _new_slots = (int *) malloc(MAX_SENSORS * sizeof(int));
    _input_sensor = input;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = (int) _input_sensor,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };
    
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &_bus));
    ESP_LOGI(TAG, "Bus initialized on GPIO %d", (int) _input_sensor);
}

void Sensor_temperature::reinitBus() {
    ESP_LOGW(TAG, "Bus reinitialization...");
    
    if (_bus) {
        onewire_bus_del(_bus);
        _bus = nullptr;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = (int) _input_sensor,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };
    
    esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &_bus);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Bus reinitialized");
    } else {
        ESP_LOGE(TAG, "Error in bus reinitialization");
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

int Sensor_temperature::scan() {
    onewire_device_iter_handle_t iter = nullptr;
    onewire_device_t device;
    esp_err_t search_result;
    
    search_result = onewire_new_device_iter(_bus, &iter);
    if (search_result != ESP_OK) {
        ESP_LOGE(TAG, "Error in iterator creation");
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
                    
                    _new_slots[new_count] = slot;
                    new_count++;
                    
                    ESP_LOGI(TAG, "Slot %d: %016llX", slot, addr);
                }
            }
        } else if (search_result != ESP_ERR_NOT_FOUND) {
            error_count++;
            if (error_count >= max_errors) {
                ESP_LOGW(TAG, "Too many errors in scanning the bus, trying to solve by reininit it ..");
                onewire_del_device_iter(iter);
                reinitBus();
                return new_count;
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    
    onewire_del_device_iter(iter);
    ESP_LOGI(TAG, "Found %d new temperature sensors (total %d)", new_count, _device_count);
    return new_count;
}

void Sensor_temperature::removeDevice(int index) {
    if (index < 0 || index >= MAX_SENSORS || !_device_active[index]) {
        return;
    }
    
    _device_active[index] = false;
    _addresses[index] = 0;
    _device_count--;
    ESP_LOGI(TAG, "Slot %d removed", index);
}

float Sensor_temperature::readDS18B20(uint64_t address) {
    // Replicating the protocol for reading a Dallas probe (unfortunately not already implemented by a library for esp idf)


    // Reset bus
    if (onewire_bus_reset(_bus) != ESP_OK) {
        return -127.0;
    }
    
    // MATCH ROM + address
    uint8_t tx_buf[9] = {CMD_MATCH_ROM};
    memcpy(&tx_buf[1], &address, 8);
    onewire_bus_write_bytes(_bus, tx_buf, 9);
    
    // CONVERT T
    uint8_t cmd = CMD_CONVERT_T;
    onewire_bus_write_bytes(_bus, &cmd, 1);
    
    // Wait for convertion
    vTaskDelay(pdMS_TO_TICKS(800));
    
    // Reset
    onewire_bus_reset(_bus);
    
    // MATCH ROM
    onewire_bus_write_bytes(_bus, tx_buf, 9);
    
    // READ SCRATCHPAD
    cmd = CMD_READ_SCRATCHPAD;
    onewire_bus_write_bytes(_bus, &cmd, 1);
    
    // read 9 byte
    uint8_t data[9];
    if (onewire_bus_read_bytes(_bus, data, 9) != ESP_OK) {
        return -127.0;
    }
    
    // Calculates temperature
    int16_t raw = (data[1] << 8) | data[0];
    return (float)raw / 16.0;
}

float Sensor_temperature::readTemperature(int index) {
    if (index < 0 || index >= MAX_SENSORS) {
        ESP_LOGE(TAG, "Illegal index: %d", index);
        return -127.0;
    }
    
    if (!_device_active[index]) {
        ESP_LOGW(TAG, "Not found a probe on lot %d, trying to scan the bus for it", index);
        scan();
        if(!_device_active[index]){
            ESP_LOGW(TAG, "Not found a probe on lot %d definitely", index);
            return -127.0;
        }
    }
    
    //reading the temperature through the requested probe
    int retry_num=0;
    float temp;
    do{
        temp = readDS18B20(_addresses[index]);
        retry_num++;
        if(temp == -127){
            reinitBus();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while(temp == -127 && retry_num<5);

    if(retry_num>=5){
        ESP_LOGW(TAG, "Error in reading temperature data from the bus, trying to reinit the bus and re-reading the temperature");
    }
    
    if (temp != -127.0) {
        // temperature = -127, means an error in reading it
        ESP_LOGI(TAG, "Slot %d: %.2f °C", index, temp);
    } else{
        ESP_LOGE(TAG, "Error in reading the temperature, returning -127 instead of correct value");
    }
    
    return temp;
}