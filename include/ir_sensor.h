#ifndef IR_SENSOR_H
#define IR_SENSOR_H

void ir_sensor_init();

int read_front_ir();
int read_left_ir();
int read_right_ir();

bool is_wall_front();
bool is_Wall_left();
bool is_wall_right();

#endif