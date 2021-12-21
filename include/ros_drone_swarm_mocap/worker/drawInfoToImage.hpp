#ifndef DRAW_INFO_TO_IMAGE_HPP
#define DRAW_INFO_TO_IMAGE_HPP

#include "worker/misc.hpp"
#ifndef OUTSIDE_ROS_ENV
#include <ros/ros.h>
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"
#endif

#include <opencv2/opencv.hpp>

/**
 * \brief
 * \param img
 * 
*/
void cameraPrintInfo(cv::Mat &img, int id){
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

    resTxt = "Node Id: [" + std::to_string(id) + "]";
    cv::putText(img, resTxt, cv::Point(50, 85), cv::FONT_HERSHEY_DUPLEX, 1, greenColor, 1);
}

/**
 * \brief
 * \param img
 * \param procData
 * \param circles 
*/
#ifndef OUTSIDE_ROS_ENV
void cameraPrintInfoprocData(cv::Mat &img, const ros_drone_swarm_mocap::mocap_worker_data& procData){
    cameraPrintInfo(img, procData.nodeID);
}
#endif

/**
 * \brief 
 * 
 * \param img
 * \param outing
 * \param procData
 */
#ifndef OUTSIDE_ROS_ENV
void drawCircles(cv::Mat img, cv::Mat& outimg, const ros_drone_swarm_mocap::mocap_worker_data procData){
    outimg = img.clone();
    for( uint k = 0; k < procData.balls.size(); k++){
        cv::Scalar blueColor(255, 0, 0);
        cv::Scalar redColor(0, 0, 255);

        ros_drone_swarm_mocap::detected_ball_data bd = procData.balls[k];
        
        int x = bd.image_plane_x;
        int y = bd.image_plane_y;
        float r = bd.image_plane_r;
        float d = bd.distance_from_camera;
        float ax = bd.xangle;
        float ay = bd.yangle;
        cv::Point center(x, y);
        
        // circle center
        cv::circle(outimg, center, 1, redColor, 2, cv::LINE_AA);
        // cv::circle(outimg, center, r, blueColor, 2, cv::LINE_AA);
        cv::rectangle(outimg, cv::Rect(x - r, y - r, r*2, r*2), blueColor,2);

        std::string textD = "Distance: " + std::to_string(d);
        std::string textAx = "Angle x: " + std::to_string(ax);
        std::string textAy = "Angle y: " + std::to_string(ay);
        cv::putText(outimg, textD, cv::Point(x-r, y-r-50), cv::FONT_HERSHEY_DUPLEX, 0.7, redColor, 1);
        cv::putText(outimg, textAx, cv::Point(x-r, y-r-30), cv::FONT_HERSHEY_DUPLEX, 0.7, redColor, 1);
        cv::putText(outimg, textAy, cv::Point(x-r, y-r-10), cv::FONT_HERSHEY_DUPLEX, 0.7, redColor, 1);
    }
}
#endif

#endif //DRAW_INFO_TO_IMAGE_HPP