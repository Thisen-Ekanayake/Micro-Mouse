#ifndef MAZE_H
#define MAZE_H

#include <vector>
#include <stdint.h>

namespace maze {

enum Dir { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

struct Pose {
  int x;
  int y;
  Dir d;
};

void begin(); // call from setup()
void startExploration(); // blocks while exploring
std::vector<Pose> computeShortestPath(int sx, int sy, int tx, int ty);
void executePath(const std::vector<Pose>& path);
void runToGoal(int sx, int sy, int tx, int ty); // convenience: plan+exec

// tuning setters (optional)
void setFrontThresholdMm(uint16_t mm);
void setSideThresholdMm(uint16_t mm);

} // namespace maze

#endif
