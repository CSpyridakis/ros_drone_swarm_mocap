#ifndef EXTEND_DIST_ANG_HPP
#define EXTEND_DIST_ANG_HPP

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

/**
 */
float calculateSensorSize(int objectSizeInPixels, float objectsDistanceFromCameraInMeters, 
                        ros_drone_swarm_mocap::mocap_worker_data& procData);

/**
 * \brief
 * 
 * \param objectSizeInPixels
 * \param procData
 * 
 * \return 
 */
float calculateDistanceWithDataStruct(int objectSizeInPixels, ros_drone_swarm_mocap::mocap_worker_data& procData);


/**
 * \brief
 * 
 * \param circles
 * \param procData
 */
void saveDistancesToProcData(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData, const std::vector<double> inCircleLedDur = std::vector<double>());

void fixCenterRadius(std::vector<cv::Vec3f> &circles);

#endif //EXTEND_DIST_ANG
