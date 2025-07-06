#ifndef IR_SENSOR_H
#define IR_SENSOR_H

void ir_sensor_init();

int read_ir_front();
int read_ir_left();
int read_ir_right();

bool is_wall_front();
bool is_wall_left();
bool is_wall_right();

#endif