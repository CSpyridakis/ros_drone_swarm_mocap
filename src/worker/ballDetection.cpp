#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <iostream>

// Messages
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

// Detection
#include "worker/ballDetection.hpp"
#include "worker/drawInfoToImage.hpp"
#include "worker/houghDetection.hpp"
#include "worker/hsvDetection.hpp"

float calculateDistanceFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter ){

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / ((float)objectsSizeInPixels * sensorSizeInMillileter) );
}

float calculateDistanceWithDataStruct(int objectSizeInPixels, ros_drone_swarm_mocap::mocap_worker_data& procData){
    return calculateDistanceFull(procData.focalLengthInMillimeters, procData.objectsRealSizeInMeter, 
                                    procData.imageSizeInPixels, objectSizeInPixels, procData.sensorSizeInMillimeters);
}

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
 * \brief: ROS image transfer needs CV_8UC3 to display image in web server, so fix Mat in other case
 */
void fixMatForImageTransfer(cv::Mat &img){
    if(img.type() == CV_8UC1){
        std::vector<cv::Mat> copies{img, img, img};  
        cv::merge(copies,img);
    }
}


// =========================================================================================================


void detectBall(const cv::Mat img, cv::Mat& imgOut, ros_drone_swarm_mocap::mocap_worker_data& procData){
    cv::Mat imgProcDebug = img.clone();
    cv::Mat imgTmp = img.clone();

    std::vector<cv::Vec3f> circles;
    
#if DETECTION_MODE == MODE_COLOR_DETECTION
    hsvDetection(imgTmp, circles);
    imgProcDebug = imgTmp.clone();
#elif DETECTION_MODE == MODE_SHAPE_DETECTION
    houghDetection(imgTmp, circles);
#endif

    saveDistancesToProcData(circles, procData);

#ifdef DEBUG
    cameraPrintInfo(imgProcDebug, procData, circles);
    drawCircles(imgProcDebug, imgProcDebug, procData, circles);
#endif

    imgOut = imgProcDebug.clone();
    fixMatForImageTransfer(imgOut);
}


