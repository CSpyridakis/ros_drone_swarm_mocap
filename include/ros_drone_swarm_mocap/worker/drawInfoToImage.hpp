#ifndef DRAW_INFO_TO_IMAGE_HPP
#define DRAW_INFO_TO_IMAGE_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

/**
 * \brief
 * \param img
 * \param procData
 * \param circles 
*/
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

/**
 * \brief 
 * 
 * \param img
 * \param outing
 * \param procData
 * \param circles
 */
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
        
        std::stringstream stream;
        std::string x = std::to_string(c[0]);
        std::string y = std::to_string(c[1]);
        std::string R = std::to_string(c[2]);

        stream.str(std::string());
        stream << std::fixed << std::setprecision(2) << procData.balls[k].distance_from_camera;
        std::string L = stream.str();

        stream.str(std::string());
        stream << std::fixed << std::setprecision(2) << procData.balls[k].xangle;
        std::string xa = stream.str();

        stream.str(std::string());
        stream << std::fixed << std::setprecision(2) << procData.balls[k].yangle;
        std::string ya = stream.str();

        resTxt = "(" + x + "," + y + ") R " + R + " D [" + L + "]" + "<" + xa + ", " + ya + ">";
        
        cv::putText(outimg, resTxt, cv::Point(c[0]-c[2], c[1]-c[2]), cv::FONT_HERSHEY_DUPLEX, 0.8, redColor, 1);
    }
}

#endif //DRAW_INFO_TO_IMAGE_HPP