#include "worker/hsvDetection.hpp"

int gaussian_kernel_size = 5;
int minV=0;
int maxV=120;

int minH = 0;
int maxH = 80;

int minS = 100;
int maxS = 170;

void updateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr& msg){
    if (msg->id == 1){  //TODO: change 
        // if input gaussian kernel size is possitive and 
        gaussian_kernel_size = (msg->gaussian_kernel_size > 0 && msg->gaussian_kernel_size% 2 != 0) ? msg->gaussian_kernel_size : gaussian_kernel_size ;
        minV = msg->minV;
        maxV = msg->maxV;
        minH = msg->minH;
        maxH = msg->maxH;
        minS = msg->minS;
        maxS = msg->maxS;  
        ROS_INFO("Got new HSV values Min(%d, %d, %d), Max(%d, %d, %d)", minH, minS, minV, maxH, maxS, maxV);
    }
}

void hsvDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles){
    cv::GaussianBlur(img, img, cv::Size(gaussian_kernel_size, gaussian_kernel_size), 5, 0);
    cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
    cv::inRange(img, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), img);

    // imgProcDebug = imgTmp.clone();
}