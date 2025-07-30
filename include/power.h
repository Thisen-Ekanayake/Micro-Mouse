#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

bool initPowerSensor();
float getBusVoltage(uint8_t channel);
float getShuntVoltage(uint8_t channel);
float getCurrent(uint8_t channel);

#endif