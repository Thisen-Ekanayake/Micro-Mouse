#include "floodfill.h"
#include "config.h"
#include "maze.h"

#include <stdint.h>

#define MAX_COST 255

static uint8_t flood[MAZE_SIZE][MAZE_SIZE];

void floodfill_init() {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            flood[y][x] = MAX_COST;
        }
    }
}

typedef struct {
    int x, y;
} Cell;

#define QUEUE_SIZE (MAZE_SIZE * MAZE_SIZE)
static Cell queue[QUEUE_SIZE];
static int q_front = 0;
static int q_back = 0;

void enqueue(int x, int y) {
    queue[q_back++] = (Cell){x,y};
    if (q_back >= QUEUE_SIZE) q_back = 0;
}

Cell dequeue() {
    Cell c = queue[q_front++];
    if (q_front >= QUEUE_SIZE) q_front = 0;
    return c;
}

int is_queue_empty() {
    return q_front == q_back;
}

void floodfill_update(int goal_x, int goal_y) {
    floodfill_init(); // reset all costs
    q_front = q_back = 0;

    flood[goal_y][goal_x] = 0;
    enqueue(goal_x, goal_y);

    while (!is_queue_empty()) {
        Cell c = dequeue();
        int cost = flood[c.y][c.x];

        for (Direction d = NORTH; d <= WEST; d++) {
            int nx = c.x;
            int ny = c.y;

            switch (d) {
                case NORTH: ny -= 1; break;
                case EAST:  nx += 1; break;
                case SOUTH: ny += 1; break;
                case WEST:  nx -= 1; break; 
            }

            if (!is_valid_cell(nx, ny)) continue;
            if (has_Wall(c.x, c.y, d)) continue;

            if (flood[ny][nx] > cost + 1) {
                flood[ny][nx] = cost + 1;
                enqueue(nx, ny);
            }
        }
    }
}

int floodfill_get_cost(int x, int y) {
    if (!is_valid_cell(x, y)) return MAX_COST;
    return flood[y][x];
}

Direction floodfill_get_lowest_neighbor(int x, int y) {
    int min_cost = MAX_COST;
    Direction best_dir = NORTH;

    for (Direction d = NORTH; d <= WEST; d++) {
        int nx = x, ny = y;
        switch (d) {
            case NORTH: ny -= 1; break;
            case EAST:  nx += 1; break;
            case SOUTH: ny += 1; break;
            case WEST:  nx -= 1; break;
        }

        if (!is_valid_cell(nx, ny)) continue;
        if (has_Wall(x, y, d)) continue;

        int cost = floodfill_get_cost(nx, ny);
        if (cost < min_cost) {
            min_cost = cost;
            best_dir = d;
        }
    }

    return best_dir;
}

/*

Example: Solve Toward Center

floodfill_update(7, 7); // Or (7,8) (8,7) (8,8) — adjust depending on goal center

while (x != goal_x || y != goal_y) {
    Direction next = floodfill_get_lowest_neighbor(x, y);
    move_in_direction(next);
    x += dx[next];
    y += dy[next];
}

*/