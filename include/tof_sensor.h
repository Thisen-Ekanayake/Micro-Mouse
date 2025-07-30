#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>

void initToFSensor();
void readToF(uint16_t &d1, uint16_t &d2, uint16_t &d3, String &l, String &r, String &f);

#endif