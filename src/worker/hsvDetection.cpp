#include "worker/hsvDetection.hpp"
// #include "worker/misc.hpp"
int gaussian_kernel_size = 5;
int minH = 40;
int minS = 70;
int minV = 150;
int maxH = 60;
int maxS = 120;
int maxV = 255;
int di_er_kernel = 1;
float scale = 0.3;

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
        ROS_INFO("Got new HSV values Min(%d, %d, %d), Max(%d, %d, %d) | di_er_kernel: %d, gaussian_kernel_size: %d", minH, minS, minV, maxH, maxS, maxV, di_er_kernel, gaussian_kernel_size);
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
    cv::Mat mask, bitwise_mask, hsvImg, tmpImg = img.clone();
    cv::GaussianBlur(tmpImg, tmpImg, cv::Size(gaussian_kernel_size, gaussian_kernel_size), 5, 0);
    cv::cvtColor(tmpImg, hsvImg, cv::COLOR_BGR2HSV);
    cv::inRange(hsvImg, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);

    if (di_er_kernel % 2 == 0 ) di_er_kernel = di_er_kernel > 0 ? di_er_kernel + 1 : 1;
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
    
    copyImageTo(img, hsvImg, "HSV colorspace");
    copyImageTo(img, mask, "Mask", mask.cols);
    copyImageTo(img, bitwise_mask, "Bitwise Mask", mask.cols + 10 + bitwise_mask.cols + 10);
    // // Resize bitwise mask
    

    
    // calculateSensorSize(2*radius);
// ----------------------------------------------------------------------------------------------------------
    
}