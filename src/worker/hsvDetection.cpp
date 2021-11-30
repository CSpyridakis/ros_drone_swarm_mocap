#include "worker/hsvDetection.hpp"
// #include "worker/misc.hpp"
int gaussian_kernel_size = 5;
int minV=0;
int maxV=120;

int minH = 0;
int maxH = 80;

int minS = 100;
int maxS = 170;

int di_er_kernel = 1;

float scale = 0.3;

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

void copyImageTo(cv::Mat &img, const cv::Mat toCopyImg, const std::string text, const int left_offset = 10, const int bottom_offset = 10, const float scale = 0.3){
    cv::Mat tmpImg;
    int resC = int(toCopyImg.cols * scale);
    int resR = int(toCopyImg.rows * scale);
    cv::resize(toCopyImg, tmpImg, cv::Size(resC,resR), cv::INTER_LINEAR);
    cv::putText(tmpImg, text, cv::Point(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0,255,0), 1);
    int left_off = (left_offset + resC < img.cols) ? left_offset : left_offset * scale + 20;
    int bottom_off = (bottom_offset + resR < img.rows) ? img.rows - tmpImg.rows - 10 : bottom_offset * scale + 20;
    // ROS_INFO("[%d, %d] -> [%d, %d , %d, %d]", toCopyImg.cols, toCopyImg.rows, left_off, bottom_off, resC, resR);
    tmpImg.copyTo(img(cv::Rect(left_off, bottom_off, resC, resR)));
}

void hsvDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles){
    cv::Mat mask, bitwise_mask, tmpImg = img.clone();
    cv::GaussianBlur(tmpImg, tmpImg, cv::Size(gaussian_kernel_size, gaussian_kernel_size), 5, 0);
    cv::cvtColor(tmpImg, tmpImg, cv::COLOR_BGR2HSV);
    cv::inRange(tmpImg, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);

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
    
    copyImageTo(img, mask, "Mask");
    copyImageTo(img, bitwise_mask, "Bitwise Mask", bitwise_mask.cols);
    // // Resize bitwise mask
    

    

// ----------------------------------------------------------------------------------------------------------

}