/*
#include <Arduino.h>
#include "config.h"
#include "ir_sensor.h"

// tune threshold values later based on testing (added in config.h)

void ir_sensor_init() {
    pinMode(IR_SENSOR_FRONT, INPUT);
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);
}

int read_ir_front() {
    return analogRead(IR_SENSOR_FRONT);
}

int read_ir_left() {
    return analogRead(IR_SENSOR_LEFT);
}

int read_ir_right() {
    return analogRead(IR_SENSOR_RIGHT);
}

bool is_wall_front() {
    return read_ir_front() > IR_THRESHOLD_FRONT;
}

bool is_wall_left() {
    return read_ir_left() > IR_THRESHOLD_LEFT;
}

bool is_wall_right() {
    return read_ir_right() > IR_THRESHOLD_RIGHT;
}

/*
===================================================================================================

add this to loop in main to test

void loop() {
    Serial.printf("Front: %d | Left: %d | Right: %d\n",
        read_ir_front(),
        read_ir_left(),
        read_ir_right());

    if (is_wall_front()) Serial.println("Wall Ahead!");
    if (is_wall_left()) Serial.println("Wall on Left!");
    if (is_wall_right()) Serial.println("Wall on Right!");
    
    delay(500);
}

===================================================================================================

Tuning IR Thresholds (Later with Hardware)
Print the raw values at various distances:

At ~4 cm (real wall): log the ADC value
At open space (>40 cm): log again

Set IR_THRESHOLD_X values somewhere in between

===================================================================================================
*/