#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

// high-level movement APIs
void move_forward_cm(float cm);
void move_backward_cm(float cm);
void rotate_90_left();
void rotate_90_right();

#ifdef __cplusplus
}
#endif
#endif