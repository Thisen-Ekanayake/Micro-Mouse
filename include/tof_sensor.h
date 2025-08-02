#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

void initToFSensor();
void readToF(uint16_t &d1, uint16_t &d2, uint16_t &d3, String &l, String &r, String &f);

#ifdef __cplusplus
}
#endif

#endif