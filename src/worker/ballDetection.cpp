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
#include "worker/misc.hpp"

void detectBall(const cv::Mat img, cv::Mat& imgOut, ros_drone_swarm_mocap::mocap_worker_data& procData){
    procData.balls.clear();
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

    // ROS_INFO("%f", calculateSensorSize(55, 0.5, procData));

#ifdef DEBUG
    cameraPrintInfo(imgProcDebug, procData, circles);
    // drawCircles(imgProcDebug, imgProcDebug, procData, circles);
#endif

    imgOut = imgProcDebug.clone();
    fixMatForImageTransfer(imgOut);
}


