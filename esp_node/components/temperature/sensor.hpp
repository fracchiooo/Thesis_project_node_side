#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "onewire_bus.h"
#include "onewire_device.h"

#define MAX_SENSORS 1

class Sensor_temperature {
private:
    onewire_bus_handle_t _bus;
    uint64_t _addresses[MAX_SENSORS];
    bool _device_active[MAX_SENSORS];
    int _device_count;
    gpio_num_t _input_sensor;
    int* _new_slots;
    
    int findFreeSlot();
    int findDeviceByAddress(uint64_t address);
    float readDS18B20(uint64_t address);
    void reinitBus();

public:
    Sensor_temperature();
    ~Sensor_temperature();
    int* getSlots() const {return _new_slots;}
    void begin(gpio_num_t input);
    int scan();
    void removeDevice(int index);
    float readTemperature(int index);
    int getDeviceCount() const { return _device_count; }
};
