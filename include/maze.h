#ifndef MAZE_H
#define MAZE_H
#include <stdint.h>

typedef struct {
    uint8_t walls;
    uint8_t visited;
    uint8_t distance;
} Cell;

void init_maze();
Cell* get_cell(uint8_t x, uint8_t y);
void set_wall(uint8_t x, uint8_t y, char dir);
void print_maze();

#endif