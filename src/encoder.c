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


volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;

void IRAM_ATTR left_encoder_isr() {
    bool a = digitalRead(ENCODER_LEFT_A);
    bool b = digitalRead(ENCODER_LEFT_B);
    left_ticks += (a == b) ? 1 : -1;
}

void IRAM_ATTR right_encoder_isr() {
    bool a = digitalRead(ENCODER_RIGHT_A);
    bool b = digitalRead(ENCODER_RIGHT_B);
    right_ticks += (a == b) ? 1 : -1;
}

void encoder_init() {
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

    /*
    Interrupt Flooding:
    when attaching interrupts to both A and B channels of each encoder this happens thats why one encoder worked properly but the other didnt
    This doubles the ISR load, possibly overwhelming the CPU or causing missed interrupts (especially on noisy channels or poor pins).

    solution: use only one channel per encoder and this reduce the pressure but cutting down interrupts 50%
    */
   
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), left_encoder_isr, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), right_encoder_isr, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), right_encoder_isr, CHANGE);
}

void reset_ticks() {
    noInterrupts();
    left_ticks = 0;
    right_ticks = 0;
    interrupts();
}

int32_t get_left_ticks() {
    noInterrupts();
    int32_t ticks = left_ticks;
    interrupts();
    return ticks;
}

int32_t get_right_ticks() {
    noInterrupts();
    int32_t ticks = right_ticks;
    interrupts();
    return ticks;
}

int32_t get_avg_ticks() {
    return (get_left_ticks() + get_right_ticks()) / 2;
}