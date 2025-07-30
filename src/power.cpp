#include "power.h"
#include <Adafruit_INA3221.h>

Adafruit_INA3221 ina3221 = Adafruit_INA3221();

bool initPowerSensor() {
    return ina3221.begin();
}

float getBusVoltage(uint8_t channel) {
    return ina3221.getBusVoltage(channel);
}

float getShuntVoltage(uint8_t channel) {
    return ina3221.getShuntVoltage(channel);
}

float getCurrent(uint8_t channel) {
    return ina3221.getCurrentAmps(channel);
}