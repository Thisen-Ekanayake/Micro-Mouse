#ifndef ENCODER_H
#define ENCODER_H

void encoder_init();
void reset_ticks();
int get_left_ticks();
int get_right_ticks();
int get_avg_ticks();

#endif