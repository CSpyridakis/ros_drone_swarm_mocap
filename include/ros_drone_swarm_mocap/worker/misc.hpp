#ifndef MISC_HPP
#define MISC_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

#define DEBUG_FUNCTIONS
#define DEBUG_IMAGE_PRINTS

/**
 * 
 */
float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis);

/**
 */
float calculateSensorsSizeFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float objectsDistanceFromCameraInMeters);

/**
 */
float calculateSensorSize(int objectSizeInPixels, float objectsDistanceFromCameraInMeters, 
                        ros_drone_swarm_mocap::mocap_worker_data& procData);

/**
 * \brief
 * 
 * \param focalLengthInMillimeter
 * \param objectsRealSizeInMeter
 * \param imageSizeInPixels
 * \param objectsSizeInPixels
 * \param sensorSizeInMillimeter
 * 
 * \return 
 */
float calculateDistanceFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter );


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
void saveDistancesToProcData(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData);



/**
 * \brief ROS image transfer needs CV_8UC3 to display image in web server, so fix Mat in other case
 * 
 * \param img
 */
void fixMatForImageTransfer(cv::Mat &img);

/**
 * \brief
 * 
 * \param img1
 * \param img2
 * \param outimage
 */
void combineImages(cv::Mat img1, cv::Mat img2, cv::Mat &outimage);


void copyImageTo(cv::Mat &img, const cv::Mat toCopyImg, const std::string text, const int left_offset, const int bottom_offset, const float scale);

#endif //MISC_HPP