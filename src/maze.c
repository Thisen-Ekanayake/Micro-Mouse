#include <stdio.h>
#include "maze.h"
#include "config.h"

static uint8_t maze[MAZE_SIZE][MAZE_SIZE]; // each cell stores 4 wall bits

void maze_init() {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            maze[y][x] = 0;
        }
    }
}

int is_valid_cell(int x, int y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

void set_wall(int x, int y, Direction dir) {
    if (!is_valid_cell(x,y)) return;

    maze[y][x] |= (1 << dir); // set wall for this cell

    // automatically set wall on the adjacent cell
    int nx = x, ny = y;
    Direction opposite;

    switch (dir) {
        case NORTH: ny -= 1; opposite = SOUTH; break;
        case SOUTH: ny += 1; opposite = NORTH; break;
        case EAST: nx += 1; opposite = WEST; break;
        case WEST: nx -= 1; opposite = EAST; break;
    }

    if (is_valid_cell(nx, ny)) {
        maze[ny][nx] |= (1 << opposite);
    }
}

int has_wall(int x, int y, Direction dir) {
    if (!is_valid_cell(x, y)) return 0;
    return (maze[y][x] & (1 << dir)) != 0;
}

void print_maze() {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            printf("[%X]", maze[y][x]);
        }
        printf("\n");
    }
}

/*

example usage

maze_init();
set_wall(0, 0, NORTH);
if (has_wall(0, 0, NORTH)) Serial.println("Wall on North side!");

*/