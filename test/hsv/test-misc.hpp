#ifndef TEST_MISC_HPP
#define TEST_MISC_HPP

/**
 * IMPORTANT! You have to have all ROS_INFO commented out to use this code!
*/

#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <cstring>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "worker/extendDistAng.hpp"
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "worker/hsvDetection.hpp"
#include "worker/drawInfoToImage.hpp"

#define IMAGE_W 1280
#define IMAGE_H 720

static float objRealSize_m =  0.206; // 0.144;   // TODO: You may need to change this value

// For undistortion 1280,720
static float cameraCalibrationdata[9] = {9.113812935295416e+02, 0.651033616843436, 6.644831723997970e+02, 0, 9.113086377881884e+02, 3.713670194501918e+02,  0, 0, 1};
static float distCoeffsCalibrationdata[5] = {-0.032436584225176, 0.087146504956371, -4.167775601669913e-04, -4.631801852683015e-04, -0.073076772853399};
cv::Mat camCalib = cv::Mat(3, 3, CV_32F, cameraCalibrationdata);
cv::Mat distCoef = cv::Mat(1, 5, CV_32F, distCoeffsCalibrationdata);
// =========================================================================
static float xsensorsize_mm = 1355.31;//1414.24; //1354.724121; //1269.375; //1354.724121;
static float ysensorsize_mm = 1354.724121;

void procDataParams(ros_drone_swarm_mocap::mocap_worker_data &procData){
    procData.nodeID = 0;
    procData.camera.imageHeightInPixels = IMAGE_H;
    procData.camera.imageWidthInPixels =IMAGE_W;
    procData.camera.XfocalLengthInMillimeters = cameraCalibrationdata[0];
    procData.camera.YfocalLengthInMillimeters = cameraCalibrationdata[4];
    procData.camera.XFieldOfViewInAngles = 2 * atan((float)procData.camera.imageWidthInPixels  / (2 * (float)procData.camera.XfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.YFieldOfViewInAngles = 2 * atan((float)procData.camera.imageHeightInPixels / (2 * (float)procData.camera.YfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.objectsRealSizeInMeter = objRealSize_m;
    procData.camera.XsensorSizeInMillimeters = xsensorsize_mm;
    procData.camera.YsensorSizeInMillimeters = ysensorsize_mm;
    // procData.pose.x = node_x;
    // procData.pose.y = node_y;
    // procData.pose.z = node_z;
    // procData.pose.roll = node_roll;
    // procData.pose.pitch = node_pitch;
    // procData.pose.yaw = node_yaw;
}

void undDistord(cv::Mat &img, cv::Mat &Unimg){
        cv::undistort(img, Unimg, camCalib, distCoef);
}

void findBallAndDisplay(ros_drone_swarm_mocap::mocap_worker_data &procData, cv::Mat &tmpImg){
    std::vector<double> inCirclesLedDuration;
    procData.balls.clear();
    std::vector<cv::Vec3f> circles;
    hsvDetection(tmpImg, circles, inCirclesLedDuration);
    fixCenterRadius(circles);
    saveDistancesToProcData(circles, procData, inCirclesLedDuration);
    
    std::string durText = "Duration: " + std::to_string(inCirclesLedDuration[0]);

    cameraPrintInfo(tmpImg, procData.nodeID, durText);
    drawCircles(tmpImg, tmpImg, procData);
    calculateSensorSize(2*circles[0][2], 1.847, procData);
    // std::cout << "." ;  
}

void setCameraCaptureProperties(cv::VideoCapture &cap){
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));      // More fps less resolution (at least for my setup)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, IMAGE_W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMAGE_H);
    cap.set(cv::CAP_PROP_FPS, 30);
    int dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); 
    int dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps_counter = cap.get(cv::CAP_PROP_FPS);
    printf("After set, actual resolution of the video w:%d h:%d fps:%d\n", dWidth, dHeight, fps_counter); 
}

#endif //TEST_MISC_HPP