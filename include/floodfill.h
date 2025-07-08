#ifndef FLOODFILL_H
#define FLOODFILL_H

#include "maze.h"

void floodfill_init();
void floodfill_update(int goal_x, int goal_y);
int floodfill_get_cost(int x, int y);
Direction floodfill_get_lowest_neighbor(int x, int y);

#endif