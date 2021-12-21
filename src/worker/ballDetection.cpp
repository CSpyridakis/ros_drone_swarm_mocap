#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>

// Messages
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

// Detection
#include "worker/misc.hpp"
#include "worker/extendDIstAng.hpp"
#include "worker/ballDetection.hpp"
#include "worker/drawInfoToImage.hpp"
#include "worker/houghDetection.hpp"
#include "worker/hsvDetection.hpp"
#include "statistics/performance.hpp"

void detectBall(const cv::Mat img, cv::Mat& imgOut, ros_drone_swarm_mocap::mocap_worker_data& procData){
    procData.balls.clear();
    cv::Mat imgProcDebug = img.clone();
    cv::Mat imgTmp;

    std::vector<cv::Vec3f> circles;
    
    cv::Mat hsvImg = img.clone();
    cv::Mat houghImg = img.clone();

    double time_now = ros::Time::now().toSec();

#if DETECTION_MODE == MODE_COLOR_DETECTION
#ifdef DEBUG_FUNCTIONS
    D_TIME(time_now, hsvDetection(hsvImg, circles), "hsvDetection");
#else
    hsvDetection(hsvImg, circles);
#endif
    fixMatForImageTransfer(hsvImg);
    imgProcDebug = hsvImg.clone();
    // combineImages(hsvImg, cv::Mat(0,0,CV_8UC3), imgTmp);
#elif DETECTION_MODE == MODE_SHAPE_DETECTION
#ifdef DEBUG_FUNCTIONS
    D_TIME(time_now, houghDetection(houghImg, circles), "houghDetection");
#else
    houghDetection(houghImg, circles);
#endif
    // combineImages(houghImg, cv::Mat(0,0,CV_8UC3), imgTmp);
    imgProcDebug = houghImg.clone();
#endif

    saveDistancesToProcData(circles, procData);

#ifdef DEBUG
    cameraPrintInfoprocData(imgProcDebug, procData);
    drawCircles(imgProcDebug, imgProcDebug, procData);
    // combineImages(imgProcDebug, imgTmp, imgTmp);
#endif

    // combineImages(imgTmp, img, imgOut);
    // combineImages(imgTmp, cv::Mat(0,0,CV_8UC3), imgOut);
    imgOut = imgProcDebug.clone();
#ifdef DEBUG_SAVE_HW_PERFORMANCE_TO_FILES
    SAVE_FRAME(time_now, imgOut);
    D_CPU(time_now);
    D_RAM(time_now);
    D_TEMP(time_now);
    D_NET(time_now, "wlan0");     //FIXME: you may need to change this interface
                                    //IMPORTANT: This produces error if not set properly!
#endif
}


