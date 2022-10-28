#ifndef HOUGH_DETECTION_HPP
#define HOUGH_DETECTION_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ros_drone_swarm_mocap/hough_values.h"

/**
 * @brief
 * 
 * @param[in] msg
 */
void updateHoughvaluesCallback(const ros_drone_swarm_mocap::hough_values::ConstPtr& msg);


/**
 * @brief
 * 
 * @param[in] img
 * @param[in] circles
 */
void houghDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles);

#endif //HOUGH_DETECTION_HPP