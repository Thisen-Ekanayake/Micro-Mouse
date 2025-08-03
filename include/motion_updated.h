#ifndef MOTION_UPDATED_H
#define MOTION_UPDATED_H

#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "tof_sensor.h"
#include "imu.h"

class Micromouse {
public:
    enum Direction { NORTH, EAST, SOUTH, WEST };

    Micromouse();
    void begin();
    void update();
    void moveForward();
    void turnLeft();
    void turnRight();
    void stop();
    void resetHeading();  // New function

    int x, y;
    Direction currentDirection;
    bool walls[4]; // N, E, S, W

private:
    void updatePosition();
    void updateWalls();
    
    float initialHeading;
};

#endif