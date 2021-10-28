#ifndef BALL_DETECTION_HPP
#define BALL_DETECTION_HPP

#include <opencv2/opencv.hpp>
#include "ros_drone_swarm_mocap/mocap_worker_data.h"

#include "ros_drone_swarm_mocap/hough_values.h"
#include "ros_drone_swarm_mocap/hsv_values.h"

#define DEBUG

#define MODE_COLOR_DETECTION 0
#define MODE_SHAPE_DETECTION 1

#define DETECTION_MODE MODE_COLOR_DETECTION

void detectBall(const cv::Mat img, cv::Mat &imgOut, ros_drone_swarm_mocap::mocap_worker_data &procData);

void updateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr &msg);

void updateHoughvaluesCallback(const ros_drone_swarm_mocap::hough_values::ConstPtr &msg);

#endif //BALL_DETECTION_HPP