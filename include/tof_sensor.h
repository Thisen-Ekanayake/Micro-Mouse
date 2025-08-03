#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

void initToFSensor();
void readToF(uint16_t &d1, uint16_t &d2, uint16_t &d3, String &l, String &r, String &f);
void readToFDistance(uint16_t &left, uint16_t &right, uint16_t &front);

#ifdef __cplusplus
}
#endif

#endif