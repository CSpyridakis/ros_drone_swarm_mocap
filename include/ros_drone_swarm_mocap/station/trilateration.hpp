#ifndef TRILATERATION_HPP
#define TRILATERATION_HPP

#include <ros/ros.h>
#include <iostream>

typedef struct coordinates{
    double x;
    double y;
    double z;
} Point;

typedef struct ancdata{
    Point anchorLocation;
    double objectsDistance;
} AnchorData;

typedef enum trilateration_return_values{
    SUCCESS,
    NOT_ENOUGH_ANCHORS
} trilRet;

/**
 * 
 */
trilRet trilateration(std::vector<AnchorData> anchors, Point objectLocation);

#endif //TRILATERATION_HPP