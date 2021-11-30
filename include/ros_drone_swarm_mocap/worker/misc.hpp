#ifndef MISC_HPP
#define MISC_HPP

#include <ros/ros.h>
#include <opencv2/core.hpp>

#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"


/**
 * 
 */
float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis = false){
    float angle = (float)( (std::abs( (float)pixelLenght/2 - NthPixel) * fov ) / pixelLenght);  ///< Calculate angle

    if (yaxis) return (float)(NthPixel >= pixelLenght/2 ) ? -angle : angle  ;       // Calculate sign (for y axis ymiddle < 0 -> positive angles)
    return (float)(NthPixel >= pixelLenght/2 ) ? angle : -angle  ;                  // Calculate sign (for x axis xmiddle < 0 -> negative angles)
}

/**
 */
float calculateSensorsSizeFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float objectsDistanceFromCameraInMeters){
    
    // ROS_INFO("\nfocalLengthInMillimeter: %f  \nobjectsRealSizeInMeter: %f \nimageSizeInPixels: %d \nobjectsSizeInPixels: %d \nobjectsDistanceFromCameraInMeters:%f\n", focalLengthInMillimeter, objectsRealSizeInMeter, imageSizeInPixels, objectsSizeInPixels, objectsDistanceFromCameraInMeters);

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / ((float)objectsSizeInPixels * objectsDistanceFromCameraInMeters) );
}

/**
 */
float calculateSensorSize(int objectSizeInPixels, float objectsDistanceFromCameraInMeters, 
                        ros_drone_swarm_mocap::mocap_worker_data& procData){
    return calculateSensorsSizeFull(procData.XfocalLengthInMillimeters, procData.objectsRealSizeInMeter, procData.imageWidthInPixels,
                            objectSizeInPixels, objectsDistanceFromCameraInMeters);
}

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
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter ){

    // ROS_INFO("\nfocalLengthInMillimeter: %f  \nobjectsRealSizeInMeter: %f \nimageSizeInPixels: %d \nobjectsSizeInPixels: %d \nsensorSizeInMillimeter:%f\n", focalLengthInMillimeter, objectsRealSizeInMeter, imageSizeInPixels, objectsSizeInPixels, sensorSizeInMillileter);

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
    return calculateDistanceFull(procData.XfocalLengthInMillimeters, procData.objectsRealSizeInMeter, 
                                    procData.imageWidthInPixels, objectSizeInPixels, procData.XsensorSizeInMillimeters);
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
        bd.distance_from_camera = calculateDistanceWithDataStruct(2*bd.image_plane_r, procData);
        bd.xangle = calculateAngle(procData.XFieldOfViewInAngles, procData.imageWidthInPixels, bd.image_plane_x);
        bd.yangle = calculateAngle(procData.YFieldOfViewInAngles, procData.imageHeightInPixels, bd.image_plane_y, true);
        // ROS_INFO("Circle - %d Center: (%d, %d) | Distance: %f | Angle x: %f | Angle y: %f", k,  bd.image_plane_x, bd.image_plane_y,  bd.distance_from_camera,  bd.xangle, bd.yangle);
        // ROS_INFO("Sensorsize: %f\n", calculateSensorSize(2*bd.image_plane_r, 1.0, procData));
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

/**
 * \brief
 * 
 * \param img1
 * \param img2
 * \param outimage
 */
void combineImages(cv::Mat img1, cv::Mat img2, cv::Mat &outimage){
    fixMatForImageTransfer(img1);
    fixMatForImageTransfer(img2);

    int rows = img1.rows + img2.rows;
    int cols = std::max(img1.cols, img2.cols);

    cv::Mat res = cv::Mat(rows, cols, CV_8UC3);

    img1.copyTo(res(cv::Rect(0, 0, img1.cols, img1.rows)));
    img2.copyTo(res(cv::Rect(0, img1.rows, img2.cols, img2.rows)));
    outimage = res.clone();
}

#endif //MISC_HPP