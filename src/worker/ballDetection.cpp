#include <ros/ros.h>
#include <iostream>

#include "worker/ballDetection.hpp"

#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

void cameraPrintInfo(cv::Mat &img, const ros_drone_swarm_mocap::mocap_worker_data& procData, std::vector<cv::Vec3f>& circles){
    cv::Scalar greenColor(0, 255, 0);
    cv::Scalar redColor(0, 0, 255);
    cv::Scalar blueColor(255, 0, 0);
    cv::Point middleUp(int(img.cols/2), 0);
    cv::Point middleDown(int(img.cols/2), img.rows);
    cv::Point middleLeft(0, int(img.rows/2));
    cv::Point middleRight(img.cols, int(img.rows/2));
    cv::Point center(int(img.cols/2), int(img.rows/2));

    // Display Axis
    cv::line(img, middleLeft, middleRight, greenColor);     // Vertical
    cv::line(img, middleUp, middleDown, greenColor);        // Horizontal
    cv::circle(img, center, 15, greenColor, 1);             // Small circle in center

    // Auxiliary text with useful info 
    std::string resTxt = "Resolution: [" + std::to_string(img.cols) + "x" + std::to_string(img.rows) + "]";
    cv::putText(img, resTxt, cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1, greenColor, 1);

    resTxt = "Node Id: [" + std::to_string(procData.nodeID) + "]";
    cv::putText(img, resTxt, cv::Point(50, 85), cv::FONT_HERSHEY_DUPLEX, 1, greenColor, 1);
}

void drawCircles(cv::Mat img, cv::Mat& outimg, const ros_drone_swarm_mocap::mocap_worker_data procData, std::vector<cv::Vec3f>& circles){
    outimg = img.clone();
    std::string resTxt;

    for( uint k = 0; k < circles.size(); k++ ){
        cv::Scalar blueColor(255, 0, 0);
        cv::Scalar redColor(0, 0, 255);
        cv::Vec3i c = circles[k];       /// x: c[0], y:c[1], radius: c[2]
        cv::Point center(c[0], c[1]);

        // circle center
        cv::circle(outimg, center, 1, redColor, 2, cv::LINE_AA);
        // circle outline
        cv::circle(outimg, center, c[2], blueColor, 2, cv::LINE_AA);
        
        resTxt = "(" + std::to_string(c[0]) + "," + std::to_string(c[1]) + ") - " + std::to_string(c[2]) + " - [" + std::to_string(procData.balls[k].distance_from_camera) + "]";
        cv::putText(outimg, resTxt, cv::Point(c[0]-c[2], c[1]-c[2]), cv::FONT_HERSHEY_DUPLEX, 1, redColor, 1);
    }
}

// =========================================================================================================
// =========================================================================================================
// =========================================================================================================

void calculateDistance(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData){
    ros_drone_swarm_mocap::detected_ball_data bd;
    for( uint k = 0; k < circles.size(); k++ ){
        bd.image_plane_x = circles[k][0];
        bd.image_plane_y = circles[k][1];
        bd.image_plane_r = circles[k][2];
        bd.distance_from_camera = 0;
        // TODO: Calculate distance
    }
    procData.balls.push_back(bd);
}

// =========================================================================================================


void detectBall(const cv::Mat img, cv::Mat& imgOut, ros_drone_swarm_mocap::mocap_worker_data& procData){
    cv::Mat imgProcDebug = img.clone();
    cv::Mat imgTmp = img.clone();

    std::vector<cv::Vec3f> circles;
    
#if DETECTION_MODE == MODE_COLOR_DETECTION
    cv::GaussianBlur(imgTmp, imgTmp, cv::Size(5,5), 5, 0);
    cv::cvtColor(imgTmp, imgTmp, cv::COLOR_BGR2HSV);
    // cv::inRange(img,(0, 0, 100),(120, 80, 170), imgTmp);
#elif DETECTION_MODE == MODE_SHAPE_DETECTION
    cv::cvtColor(imgTmp, imgTmp, cv::COLOR_BGR2GRAY); 
    cv::medianBlur(imgTmp, imgTmp, 5);
    cv::HoughCircles(imgTmp, circles, cv::HOUGH_GRADIENT,
                        0.5,                  // Accumulator resolution 
                        20,                 // Min distance between circles
                        100,                // Canny high threshold
                        120,                // Threshold for center detection
                        0, 200);            // Min and max radius

    // cv::namedWindow("name", cv::WINDOW_NORMAL);
    // cv::imshow("name", imgTmp);
    // cv::waitKey(0);
#endif

    calculateDistance(circles, procData);

#ifdef DEBUG
    cameraPrintInfo(imgProcDebug, procData, circles);
    drawCircles(imgProcDebug, imgProcDebug, procData, circles);
#endif

    // imgOut = imgProcDebug.clone();
    imgOut = imgProcDebug.clone();
}


