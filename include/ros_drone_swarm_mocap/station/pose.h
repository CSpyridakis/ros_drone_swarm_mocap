#ifndef POSE_H
#define POSE_H

typedef struct Position{
    double x;
    double y;
    double z;
} Position;

typedef struct Orientation{
    double roll;    // X-axis (roll like a cat)
    double pitch;   // Y-axis (pitcher)
    double yaw;     // Z-axis (door's yaw)
} Orientation;

typedef struct Point{
    Position pos;
    Orientation ori;
} Point;

#endif //POSE_H