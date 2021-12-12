#ifndef TRILATERATION_HPP
#define TRILATERATION_HPP

#include <ros/ros.h>
#include <iostream>

#include "pose.h"

typedef struct anchor_data{
    Point pose;                 // Anchor's pose (position + orientation)
    double objectsDistance;     // Detected object's distance 
} anchor_data;

typedef enum trilateration_return_value{
    SUCCESS = 0,
    NOT_ENOUGH_ANCHORS
} trilatRet;

/**
 * 
 */
trilatRet trilateration(const std::vector<anchor_data> anchors, Point &objectsPose);

#endif //TRILATERATION_HPP