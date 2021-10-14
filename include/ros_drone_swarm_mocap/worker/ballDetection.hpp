#ifndef BALL_DETECTION_HPP
#define BALL_DETECTION_HPP

#include <opencv2/opencv.hpp>
#include "ros_drone_swarm_mocap/mocap_worker_data.h"

#define DEBUG

void detectBall(const cv::Mat img, cv::Mat& imgOut, ros_drone_swarm_mocap::mocap_worker_data& procData);

#endif //BALL_DETECTION_HPP