#ifndef MISC_HPP
#define MISC_HPP

#include <ros/ros.h>
#include <opencv2/core.hpp>

#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"



/**
 * \brief
 * 
 * \param focalLengthInMillimeter
 * \param objectsRealSizeInMeter
 * \param imageSizeInPixels
 * \param objectsSizeInPixels
 * \param sensorSizeInMillileter
 * 
 * \return 
 */
float calculateDistanceFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter ){

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / ((float)objectsSizeInPixels * sensorSizeInMillileter) );
}



/**
 * \brief
 * 
 * \param objectSizeInPixels
 * \param procData
 * 
 * \return 
 */
float calculateDistanceWithDataStruct(int objectSizeInPixels, ros_drone_swarm_mocap::mocap_worker_data& procData){
    return calculateDistanceFull(procData.focalLengthInMillimeters, procData.objectsRealSizeInMeter, 
                                    procData.imageSizeInPixels, objectSizeInPixels, procData.sensorSizeInMillimeters);
}



/**
 * \brief
 * 
 * \param circles
 * \param procData
 */
void saveDistancesToProcData(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData){
    ros_drone_swarm_mocap::detected_ball_data bd;
    for( uint k = 0; k < circles.size(); k++ ){
        bd.image_plane_x = circles[k][0];
        bd.image_plane_y = circles[k][1];
        bd.image_plane_r = circles[k][2];
        bd.distance_from_camera = calculateDistanceWithDataStruct(circles[k][2], procData);
    }
    procData.balls.push_back(bd);
}



/**
 * \brief ROS image transfer needs CV_8UC3 to display image in web server, so fix Mat in other case
 * 
 * \param img
 */
void fixMatForImageTransfer(cv::Mat &img){
    if(img.type() == CV_8UC1){
        std::vector<cv::Mat> copies{img, img, img};  
        cv::merge(copies,img);
    }
}

#endif //MISC_HPP