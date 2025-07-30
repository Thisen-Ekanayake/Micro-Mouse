#include "display.h"
#include "config.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        while(true) delay (10);
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
}

void updateDisplay(float v0, float i0, float v1, float i1, const String& l, const String& r, const String& f, bool charging) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);

    display.print(F("OUT V: ")); display.print(v0, 2);
  display.print(F("  I: ")); display.print(i0 * 1000, 0); display.println(F("mA"));

  display.print(F("BAT V: ")); display.print(v1, 2);
  display.print(F(" I: ")); display.print(i1 * 1000, 0); display.println(F("mA"));

  display.setCursor(0, 24);
  display.print("Left : "); display.println(l);
  display.print("Right: "); display.println(r);
  display.print("Front: "); display.println(f);

  display.setCursor(0, 56);
  display.setTextSize(1);
  display.println(charging ? F("Charging") : F("Charger Disconnected"));

  display.display();
}