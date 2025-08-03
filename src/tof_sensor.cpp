#include "tof_sensor.h"
#include "config.h"
#include <VL53L0X.h>

VL53L0X sensor1, sensor2, sensor3;

void initToFSensor() {
  pinMode(XSHUT1, OUTPUT); pinMode(XSHUT2, OUTPUT); pinMode(XSHUT3, OUTPUT);
  digitalWrite(XSHUT1, LOW); digitalWrite(XSHUT2, LOW); digitalWrite(XSHUT3, LOW);
  delay(10);

  digitalWrite(XSHUT1, HIGH); delay(10); sensor1.init(); sensor1.setAddress(0x30);
  digitalWrite(XSHUT2, HIGH); delay(10); sensor2.init(); sensor2.setAddress(0x31);
  digitalWrite(XSHUT3, HIGH); delay(10); sensor3.init(); sensor3.setAddress(0x32);

  sensor1.setTimeout(500); sensor2.setTimeout(500); sensor3.setTimeout(500);
  sensor1.startContinuous(); sensor2.startContinuous(); sensor3.startContinuous();
}

void readToF(uint16_t &d1, uint16_t &d2, uint16_t &d3, String &l, String &r, String &f) {
  d1 = sensor1.readRangeContinuousMillimeters();
  d2 = sensor2.readRangeContinuousMillimeters();
  d3 = sensor3.readRangeContinuousMillimeters();

  l = sensor1.timeoutOccurred() ? "Timeout" : String(d1) + " mm";
  r = sensor2.timeoutOccurred() ? "Timeout" : String(d2) + " mm";
  f = sensor3.timeoutOccurred() ? "Timeout" : String(d3) + " mm";
}

void readToFDistance(uint16_t &left, uint16_t &right, uint16_t &front) {
  left = sensor1.readRangeContinuousMillimeters();
  if (sensor1.timeoutOccurred()) left = 10000;  // large value indicate no wall

  right = sensor2.readRangeContinuousMillimeters();
  if (sensor2.timeoutOccurred()) right = 10000;

  front = sensor3.readRangeContinuousMillimeters();
  if (sensor3.timeoutOccurred()) front = 10000;
}
