#include <Arduino.h>
#include "encoder.h"

volatile int left_ticks = 0;
volatile int right_ticks = 0;

void IRAM_ATTR left_encoder_isr() {
    left_ticks++;
}
void IRAM_ATTR right_encoder_isr() {
    right_ticks++;
}

void encoder_init() {
    attachInterrupt(digitalPinToInterrupt(35), left_encoder_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(36), right_encoder_isr, RISING);
}

int get_left_ticks() { return left_ticks; }
int get_right_ticks() { return right_ticks; }
void reset_ticks() { left_ticks = 0; right_ticks = 0; }
