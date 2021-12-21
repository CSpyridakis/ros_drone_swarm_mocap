#ifndef HSV_DETECTION_HPP
#define HSV_DETECTION_HPP

#include "worker/misc.hpp"

#ifndef OUTSIDE_ROS_ENV
#include <ros/ros.h>
#include "ros_drone_swarm_mocap/hsv_values.h"
#endif

#include <opencv2/opencv.hpp>


#ifndef OUTSIDE_ROS_ENV
/**
 * \brief 
 * 
 * \param msg
 */
void updateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr& msg);
#endif

/**
 * \brief
 * 
 * \param circles
 */
void hsvDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles);

#endif //HSV_DETECTION_HPP