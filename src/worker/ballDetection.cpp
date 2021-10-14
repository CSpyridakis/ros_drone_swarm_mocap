#include <ros/ros.h>
#include <iostream>

#include "worker/ballDetection.hpp"

void cameraPrintInfo(cv::Mat &img, const ros_drone_swarm_mocap::mocap_worker_data& procData){
    cv::Scalar greenColor(0, 255, 0);
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

void drawCircles(cv::Mat img, cv::Mat& outimg, std::vector<cv::Vec3f>& circles){
    outimg = img.clone();
    
    for( uint k = 0; k < circles.size(); k++ ){
        cv::Scalar blueColor(255, 0, 0);
        cv::Scalar redColor(0, 0, 255);
        cv::Vec3i c = circles[k];       /// x: c[0], y:c[1], radius: c[2]
        cv::Point center(c[0], c[1]);

        // circle center
        cv::circle(outimg, center, 1, redColor, 2, cv::LINE_AA);
        // circle outline
        cv::circle(outimg, center, c[2], blueColor, 2, cv::LINE_AA);
    }
}

// =========================================================================================================
// =========================================================================================================
// =========================================================================================================

void shapeDetection(cv::Mat& img, std::vector<cv::Vec3f>& circles){
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); 
    cv::medianBlur(img, img, 5);
    cv::HoughCircles(img, circles, cv::HOUGH_GRADIENT,
                        2,                  // Accumulator resolution 
                        img.cols,           // Min distance between circles
                        100,                // Canny high threshold
                        100,                // Threshold for center detection
                        0, 200);            // Min and max radius
}

void colorDetection(cv::Mat& img, std::vector<cv::Vec3f>& circles){
    cv::GaussianBlur(img, img, cv::Size(5,5), 5, 0);
    cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
    // cv::inRange(img,(0, 0, 100),(120, 80, 170), img);
}

void calculateDistance(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData){

}

// =========================================================================================================


void detectBall(const cv::Mat img, cv::Mat& imgOut, ros_drone_swarm_mocap::mocap_worker_data& procData){
    cv::Mat imgTmp = img.clone();
    std::vector<cv::Vec3f> circles;

#if defined(SHAPE_DETEC)
    cv::Mat imgShape = img.clone();
    shapeDetection(imgShape, circles);
    imgTmp = imgShape.clone();
#elif defined(COLOR_DETEC)
    cv::Mat imgColor = img.clone();
    colorDetection(imgColor, circles);
    imgTmp = imgColor.clone();
#endif

#ifdef DEBUG
    cameraPrintInfo(imgTmp, procData);
    drawCircles(imgTmp, imgTmp, circles);
#endif

    imgOut = imgTmp.clone();
    calculateDistance(circles, procData);
}


