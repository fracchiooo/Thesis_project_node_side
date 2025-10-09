#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "onewire_bus.h"
#include "onewire_device.h"

#define ONEWIRE_GPIO GPIO_NUM_26
#define MAX_SENSORS 10

class Sensor_temperature {
private:
    onewire_bus_handle_t _bus;
    uint64_t _addresses[MAX_SENSORS];
    bool _device_active[MAX_SENSORS];
    int _device_count;
    
    int findFreeSlot();
    int findDeviceByAddress(uint64_t address);
    float readDS18B20(uint64_t address);
    void reinitBus();

public:
    Sensor_temperature();
    ~Sensor_temperature();
    
    void begin();
    int scan(int* new_slots);
    void removeDevice(int index);
    float readTemperature(int index);
    int getDeviceCount() const { return _device_count; }
};
