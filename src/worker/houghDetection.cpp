#include "worker/houghDetection.hpp"

int accumulator_resolution = 2;
int min_distance_between_circles = 500;
int canny_high_threshold = 120;
int threshold_for_center_detection = 100;  

void updateHoughvaluesCallback(const ros_drone_swarm_mocap::hough_values::ConstPtr& msg){
    if(msg->id == 1){   //TODO: change
        accumulator_resolution = msg->acc_res ;
        min_distance_between_circles = msg->min_dist_betw_cir ;
        canny_high_threshold = msg->canny_high_res ;
        threshold_for_center_detection = msg->threshold_for_center ;
        ROS_INFO("Got new Hough values acc:[%d] min_dist:[%d] canny:[%d] thres:[%d]", accumulator_resolution, min_distance_between_circles, canny_high_threshold, threshold_for_center_detection);
    }
}

void houghDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles){
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); 
    cv::medianBlur(img, img, 5);
    cv::HoughCircles(img, circles, cv::HOUGH_GRADIENT,
                accumulator_resolution,         // Accumulator resolution 
                min_distance_between_circles,   // Min distance between circles
                canny_high_threshold,           // Canny high threshold
                threshold_for_center_detection, // Threshold for center detection
                0, 200);                        // Min and max radius

    // cv::namedWindow("name", cv::WINDOW_NORMAL);
    // cv::imshow("name", imgTmp);
    // cv::waitKey(0);
}