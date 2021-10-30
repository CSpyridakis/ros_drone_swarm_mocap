#ifndef BALL_DETECTION_HPP
#define BALL_DETECTION_HPP

#include <opencv2/opencv.hpp>
#include "ros_drone_swarm_mocap/mocap_worker_data.h"

// If you want to disable debug mode comment out this line 
#define DEBUG

// Available detection modes
#define MODE_COLOR_DETECTION 0
#define MODE_SHAPE_DETECTION 1

#define DETECTION_MODE MODE_COLOR_DETECTION

/**
 * \brief Detect ball in camera plane
 * 
 * \param img is the input Mat in which you want to detect the ball
 * \param imgOut is the processed result in case you want to display it for debug purposes 
 * \param procData has important variables for distance estimation and will contain afterwards,
 *          for each ball detected, the processed data
 */
void detectBall(const cv::Mat img, cv::Mat &imgOut, ros_drone_swarm_mocap::mocap_worker_data &procData);

#endif //BALL_DETECTION_HPP