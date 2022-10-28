#ifndef TRILATERATION_HPP
#define TRILATERATION_HPP

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "pose.h"

#define BALL_RADIOUS        ((float) (0.144))
#define MAX_NUM_OF_ACHORS   (4)

/**
 * This struct holds the needed data that you have to pass inside @ref trilateration function
*/
typedef struct anchor_data{
    Point pose;                 ///< Anchor pose (position + orientation)
    double objectsDistance;     ///< Detected object distance 
} anchor_data;

/**
 * Return status of @ref trilateration function
 */
typedef enum trilateration_return_value{
    SUCCESS = 0,            ///< Everything worked as expected
    NOT_ENOUGH_ANCHORS,     ///< You need to provide data from more anchors
    TOO_MANY_ANCHORS        ///< The current implementation does not support this number of anchors
} trilatRet;

/**
 * @brief Provide anchors data and by the help of the trilateration algorithm estimate free node position
 * 
 * @param[in] anchors  Anchors data see @ref anchor_data for more info
 * @param[out] objectsPose  the estimated position of the object
 *
 * @see find here https://doi.org/10.26233/heallink.tuc.91531 how exactly this function works
 * @return  the status of the process see @ref trilatRet to find out all available status codes
 */
trilatRet trilateration(const std::vector<anchor_data> anchors, Point &objectsPose);

#endif //TRILATERATION_HPP