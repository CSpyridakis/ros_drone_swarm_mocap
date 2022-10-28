#ifndef EXTEND_DIST_ANG_HPP
#define EXTEND_DIST_ANG_HPP

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

/**
 * @brief Calculate the size of the sensor in case you do not know it already
 * 
 * @param[in] objectSizeInPixels This is the number of pixels that the object occupy on the image plane
 * @param[in] objectsDistanceFromCameraInMeters The distance in meters that the object is actually 
 * @param[in] procData
 * 
 * @return The size of the sensor in milli-meters
 */
float calculateSensorSize(int objectSizeInPixels, float objectsDistanceFromCameraInMeters, 
                        ros_drone_swarm_mocap::mocap_worker_data& procData);

/**
 * @brief Calculate the camera-object distance
 * 
 * @param[in] objectSizeInPixels This is the number of pixels that the object occupy on the image plane
 * @param[in] procData This data type contains all the camera parameter needed to calculate the distance
 * 
 * @return The actual camera-object distance in meters
 */
float calculateDistanceWithDataStruct(int objectSizeInPixels, ros_drone_swarm_mocap::mocap_worker_data& procData);


/**
 * @brief
 * 
 * @param[in] circles
 * @param[in] procData
 * @param[in] inCircleLedDur
 */
void saveDistancesToProcData(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData, const std::vector<double> inCircleLedDur = std::vector<double>());


/**
 * @brief
 * 
 * @param[in] circles
 */
void fixCenterRadius(std::vector<cv::Vec3f> &circles);

#endif //EXTEND_DIST_ANG
