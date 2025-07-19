#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

void encoder_init();
void reset_ticks();
int get_left_ticks();
int get_right_ticks();
int get_avg_ticks();

#ifdef __cplusplus
}
#endif

#endif