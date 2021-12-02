#include "worker/hsvDetection.hpp"
#include "worker/misc.hpp"

static int gaussian_kernel_size = 5;
static int minH = 40;
static int minS = 70;
static int minV = 150;
static int maxH = 60;
static int maxS = 120;
static int maxV = 255;
static int di_er_kernel = 1;

void updateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr& msg){
    if (msg->id == 1){  //TODO: change 
        // if input gaussian kernel size is possitive and 
        gaussian_kernel_size = (msg->gaussian_kernel_size > 0 && msg->gaussian_kernel_size% 2 != 0) ? msg->gaussian_kernel_size : gaussian_kernel_size ;
        minH = msg->minH;
        maxH = msg->maxH;
        minS = msg->minS;
        maxS = msg->maxS;  
        minV = msg->minV;
        maxV = msg->maxV;
        di_er_kernel = msg->di_er_kernel;
        if (di_er_kernel % 2 == 0 ) di_er_kernel = di_er_kernel > 0 ? di_er_kernel + 1 : 1;
        ROS_INFO("Got new HSV values Min(%d, %d, %d), Max(%d, %d, %d) | di_er_kernel: %d, gaussian_kernel_size: %d", minH, minS, minV, maxH, maxS, maxV, di_er_kernel, gaussian_kernel_size);
    }
}

void hsvDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles){
    cv::Mat mask, bitwise_mask, hsvImg, tmpImg = img.clone();
    cv::GaussianBlur(tmpImg, tmpImg, cv::Size(gaussian_kernel_size, gaussian_kernel_size), 5, 0);
    cv::cvtColor(tmpImg, hsvImg, cv::COLOR_BGR2HSV);
    cv::inRange(hsvImg, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);

    cv::Mat kernelDiER = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(di_er_kernel, di_er_kernel));
    cv::dilate(mask, mask, kernelDiER);
    cv::erode(mask, mask, kernelDiER);

    cv::Mat Points;
    cv::findNonZero(mask,Points);
    cv::Rect ballBound = boundingRect(Points);

    cv::Point ballCenter = (ballBound.br() + ballBound.tl())*0.5;
    int radius = ballBound.height > ballBound.width ? ballBound.height/2 : ballBound.width/2;
    cv::Vec3f circle(ballCenter.x, ballCenter.y, radius);
    // ROS_INFO("New Circle: (%d, %d - %d)", ballCenter.x, ballCenter.y, radius);
    circles.push_back(circle);

// ----------------------------------------------------------------------------------------------------------
    // See bitwise image
    if(mask.type() == CV_8UC1){
        std::vector<cv::Mat> copies{mask, mask, mask};  
        cv::merge(copies,mask);
    }
    cv::bitwise_and(img, mask, bitwise_mask);
    
    copyImageTo(img, hsvImg, "HSV colorspace", 10, 10, 0.3);
    copyImageTo(img, mask, "Mask", mask.cols, 10, 0.3);
    copyImageTo(img, bitwise_mask, "Bitwise Mask", mask.cols + 10 + bitwise_mask.cols + 10, 10, 0.3);
    // // Resize bitwise mask

    // calculateSensorSize(2*radius);
// ----------------------------------------------------------------------------------------------------------
    
}