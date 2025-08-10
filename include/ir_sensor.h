/*
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

// initialize all ir sensor pins
void ir_sensor_init();

// returns raw ADC values from sensors
int read_ir_front();
int read_ir_left();
int read_ir_right();

// returns true if wall is detected (based on thresholds in config.h)
bool is_wall_front();
bool is_wall_left();
bool is_wall_right();

#endif
*/