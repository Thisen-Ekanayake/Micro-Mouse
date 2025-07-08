#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

// direction enum
typedef enum {
    NORTH   = 0,
    EAST    = 1,
    SOUTH   = 2,
    WEST    = 3
} Direction;

// wall bitmask : 4 bits per cell
#define WALL_NORTH (1 << NORTH)
#define WALL_EAST (1 << EAST)
#define WALL_SOUTH (1 << SOUTH)
#define WALL_WEST (1 << WEST)

void maze_init();
void set_wall(int x, int y, Direction dir);
int has_Wall(int x, int y, Direction dir);
int is_valid_cell(int x, int y);
void print_maze();

#endif