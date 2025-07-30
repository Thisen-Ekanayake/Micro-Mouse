#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_SSD1306.h>
#include <Wire.h>

void initDisplay();
void updateDisplay(float v0, float i0, float v1, float i1, const String& l, const String& r, String& f, bool charging);

#endif