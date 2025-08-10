#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void encoder_init();
void reset_ticks();
int32_t get_left_ticks();
int32_t get_right_ticks();
int32_t get_avg_ticks();

#ifdef __cplusplus
}
#endif

#endif