#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>

void initToFSensor();
void readToF(uint8_t &d1, uint8_t &d2, uint8_t &d3, String &l, String &r, String &f);

#endif