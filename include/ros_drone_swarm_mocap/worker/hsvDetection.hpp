#ifndef HSV_DETECTION_HPP
#define HSV_DETECTION_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ros_drone_swarm_mocap/hsv_values.h"
#include "worker/misc.hpp"

#ifdef TESTING_HSV
void fixTrackers();
void createTrackers();
void setupTrackers(int gau, int iH, int xH, int nS, int xS, int nV, int xV, int dir);
#endif

/**
 * \brief 
 * 
 * \param msg
 */
void updateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr& msg);

/**
 * \brief
 * 
 * \param circles
 */
void hsvDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles);

#endif //HSV_DETECTION_HPP