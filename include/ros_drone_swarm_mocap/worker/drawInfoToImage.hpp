#ifndef DRAW_INFO_TO_IMAGE_HPP
#define DRAW_INFO_TO_IMAGE_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "worker/misc.hpp"
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"
#include "statistics/performance.hpp"

/**
 * @brief Display on top of an image frame extra information for debug purposes. 
 *        Except from the default info you can pass extra info to print
 *  
 * @param[in,out] img The image in which you want to add the overlay 
 * @param[in] id The ID of the node that will be displayed on top of the image
 * @param[in] extraText Some extra information you may need to display
 * 
*/
static void cameraPrintInfo(cv::Mat &img, int id, std::string extraText = ""){
    // Colors
    cv::Scalar  greenColor(0, 255, 0);
    cv::Scalar  redColor(0, 0, 255);
    cv::Scalar  blueColor(255, 0, 0);
    // Positions
    cv::Point   middleUp(int(img.cols/2), 0);
    cv::Point   middleDown(int(img.cols/2), img.rows);
    cv::Point   middleLeft(0, int(img.rows/2));
    cv::Point   middleRight(img.cols, int(img.rows/2));
    cv::Point   center(int(img.cols/2), int(img.rows/2));

    // Display Axis
    cv::line(img, middleLeft, middleRight, greenColor);     // Vertical
    cv::line(img, middleUp, middleDown, greenColor);        // Horizontal
    cv::circle(img, center, 15, greenColor, 1);             // Small circle in center

    // Auxiliary text with useful info 
    std::string resTxt = "Resolution: [" + std::to_string(img.cols) + "x" + std::to_string(img.rows) + "]";
    cv::putText(img, resTxt, cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1, greenColor, 1);

    resTxt = "Node Id: [" + std::to_string(id) + "]";
    cv::putText(img, resTxt, cv::Point(50, 85), cv::FONT_HERSHEY_DUPLEX, 1, greenColor, 1);

    cpu_usage cu;
    get_cpu_usage(cu);
    resTxt = "CPU: " + to_string_with_precision(cu.used*100,1) + "%";
    cv::putText(img, resTxt, cv::Point(50, 110), cv::FONT_HERSHEY_DUPLEX, 0.5, redColor, 0.5);

    ram_usage ru        = get_ram_usage();
    float ru_used_gb    = ru.used/(1024*1024*1024);
    float ru_total_gb   = ru.total/(1024*1024*1024);
    resTxt              = "Ram: " + to_string_with_precision(ru_used_gb,2) + 
                            "GB / " + to_string_with_precision(ru_total_gb,2) + "GB";
    cv::putText(img, resTxt, cv::Point(50, 130), cv::FONT_HERSHEY_DUPLEX, 0.5, redColor, 0.5);

    cv::putText(img, extraText, cv::Point(50, 150), cv::FONT_HERSHEY_DUPLEX, 0.5, redColor, 0.5);

    // net_usage nu = get_net_usage();  
    // float nu_up_mb = nu.up/(8*1024*1024);
    // float nu_down_mb = nu.down/(8*1024*1024);
    // resTxt = "Net: U: " + to_string_with_precision(nu_up_mb,3) + "Mb / D:" + to_string_with_precision(nu_down_mb,3) + "MB";
    // cv::putText(img, resTxt, cv::Point(50, 150), cv::FONT_HERSHEY_DUPLEX, 0.5, redColor, 0.5);
}


#ifndef OUTSIDE_ROS_ENV
/**
 * @brief This is actually a wrapper for @ref cameraPrintInfo function, to make easier to use this function
 * @param[in,out] img The image in which you want to add the overlay  
 * @param[in] procData The 
 * @param[in] extraText Some extra information you may need to display 
*/
static void cameraPrintInfoprocData(cv::Mat &img, const ros_drone_swarm_mocap::mocap_worker_data& procData, std::string extraText = ""){
    cameraPrintInfo(img, procData.nodeID, extraText);
}
#endif

/**
 * @brief 
 * 
 * @param[in] img
 * @param[out] outimg
 * @param[in] procData
 */
static void drawCircles(cv::Mat img, cv::Mat& outimg, const ros_drone_swarm_mocap::mocap_worker_data procData){
    outimg = img.clone();
    for( uint k = 0; k < procData.balls.size(); k++){
        cv::Scalar blueColor(255, 0, 0);
        cv::Scalar redColor(0, 0, 255);

        ros_drone_swarm_mocap::detected_ball_data bd = procData.balls[k];
        
        int x       = bd.image_plane_x;
        int y       = bd.image_plane_y;
        float r     = bd.image_plane_r;
        float d     = bd.distance_from_camera;
        float ax    = bd.xangle;
        float ay    = bd.yangle;
        int id      = bd.id;
        cv::Point center(x, y);
        
        // circle center
        cv::circle(outimg, center, 1, redColor, 2, cv::LINE_AA);
        // cv::circle(outimg, center, r, blueColor, 2, cv::LINE_AA);
        cv::rectangle(outimg, cv::Rect(x - r, y - r, r*2, r*2), blueColor,2);

        std::string textID  = "ID: "        + std::to_string(id);
        std::string textD   = "Distance: "  + std::to_string(d);
        std::string textAx  = "Angle x: "   + std::to_string(ax);
        std::string textAy  = "Angle y: "   + std::to_string(ay);
        cv::putText(outimg, textID, cv::Point(x-r, y-r-70), cv::FONT_HERSHEY_DUPLEX, 0.7, redColor, 1);
        cv::putText(outimg, textD,  cv::Point(x-r, y-r-50), cv::FONT_HERSHEY_DUPLEX, 0.7, redColor, 1);
        cv::putText(outimg, textAx, cv::Point(x-r, y-r-30), cv::FONT_HERSHEY_DUPLEX, 0.7, redColor, 1);
        cv::putText(outimg, textAy, cv::Point(x-r, y-r-10), cv::FONT_HERSHEY_DUPLEX, 0.7, redColor, 1);
    }
}

#endif //DRAW_INFO_TO_IMAGE_HPP