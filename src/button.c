#include <Arduino.h>
#include "button.h"
#include "config.h"

void button_init() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // use internal pull-up resistor
}

int button_pressed() {
    return digitalRead(BUTTON_PIN) == LOW;  // active low logic
}

/*

how to test

in main.cpp
#include "button.h"

void setup() {
    Serial.begin(115200);
    button_init();
}

void loop() {
    if (button_pressed()) {
        Serial.println("Button Pressed!");
        delay(300); // crude debounce
    }
}

*/