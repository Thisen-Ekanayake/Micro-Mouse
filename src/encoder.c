#include <Arduino.h>
#include "encoder.h"
#include "config.h"

/*
===================================================================================================

Add direction tracking (future)
If your encoders are quadrature (A+B signals per motor),
you can determine direction and increment/decrement ticks accordingly.
For now, since your motors are likely single-channel or you’re not using direction from encoder:
Stick to ++ counting only
Use motor direction to infer whether it’s forward/reverse

===================================================================================================

How to Test Later (Serial Monitor Example)

void loop() {
    Serial.printf("L: %d, R: %d\n", get_left_ticks(), get_right_ticks());
    delay(200);
}

===================================================================================================
*/


volatile int left_ticks = 0;
volatile int right_ticks = 0;

void IRAM_ATTR left_encoder_isr() {
    left_ticks++;
}

void IRAM_ATTR right_encoder_isr() {
    right_ticks++;
}

void encoder_init() {
    pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), left_encoder_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), right_encoder_isr, RISING);
}

void reset_ticks() {
    noInterrupts();
    left_ticks = 0;
    right_ticks = 0;
    interrupts();
}

int get_left_ticks() {
    noInterrupts();
    int ticks = left_ticks;
    interrupts();
    return ticks;
}

int get_right_ticks() {
    noInterrupts();
    int ticks = right_ticks;
    interrupts();
    return ticks;
}

int get_avg_ticks() {
    return (get_left_ticks() + get_right_ticks()) / 2;
}