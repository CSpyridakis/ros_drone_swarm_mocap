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
    cv::Mat imgTmp;

    std::vector<cv::Vec3f> circles;
    
    cv::Mat hsvImg = img.clone();
    cv::Mat houghImg = img.clone();

#if DETECTION_MODE == MODE_COLOR_DETECTION
    hsvDetection(hsvImg, circles);
    fixMatForImageTransfer(hsvImg);
    combineImages(hsvImg, cv::Mat(0,0,CV_8UC3), imgTmp);
#elif DETECTION_MODE == MODE_SHAPE_DETECTION
    houghDetection(houghImg, circles);
    combineImages(houghImg, cv::Mat(0,0,CV_8UC3), imgTmp);
#endif

    saveDistancesToProcData(circles, procData);

#ifdef DEBUG
    cameraPrintInfo(imgProcDebug, procData, circles);
    // drawCircles(imgProcDebug, imgProcDebug, procData, circles);
    combineImages(imgProcDebug, imgTmp, imgTmp);
#endif

    combineImages(imgTmp, img, imgOut);
    // combineImages(imgTmp, cv::Mat(0,0,CV_8UC3), imgOut);
}


