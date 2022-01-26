#include "worker/hsvDetection.hpp"
#include "worker/misc.hpp"
#include "worker/frequency.hpp"

static frequency_analysis fa(7,63,105,48,115,149,255,1);

#define FREQUENCY
#define COUNTOURS_AREA_THRESHOLD 200
#define COUNTOURS_OBJ_AREA_THRESHOLD 0

#define ALL_NODES 0

static int gaussian_kernel_size = 5;
static int minH = 40;
static int minS = 70;
static int minV = 150;
static int maxH = 60;
static int maxS = 120;
static int maxV = 255;
static int di_er_kernel = 1;

#ifdef TESTING_HSV
void createTrackers(){
    cv::namedWindow("trackbars", cv::WINDOW_AUTOSIZE); // Create Window
    cv::createTrackbar( "gaussian", "trackbars", &gaussian_kernel_size, 20);
    cv::createTrackbar( "minH", "trackbars", &minH, 255);
    cv::createTrackbar( "maxH", "trackbars", &maxH, 255);
    cv::createTrackbar( "minS", "trackbars", &minS, 255);
    cv::createTrackbar( "maxS", "trackbars", &maxS, 255);
    cv::createTrackbar( "minV", "trackbars", &minV, 255);
    cv::createTrackbar( "maxV", "trackbars", &maxV, 255); 
    cv::createTrackbar( "di_er_kernel", "trackbars", &di_er_kernel, 20); 
}

void fixTrackers(){
        if(di_er_kernel % 2 == 0)
            cv::setTrackbarPos("di_er_kernel", "trackbars", di_er_kernel+1);

        if(gaussian_kernel_size % 2 == 0)
            cv::setTrackbarPos("gaussian", "trackbars", gaussian_kernel_size+1);
}
void setupTrackers(int gau, int iH, int xH, int nS, int xS, int nV, int xV, int dir){
    cv::setTrackbarPos( "gaussian", "trackbars", gau);
    cv::setTrackbarPos( "minH", "trackbars", iH);
    cv::setTrackbarPos( "maxH", "trackbars", xH);
    cv::setTrackbarPos( "minS", "trackbars", nS);
    cv::setTrackbarPos( "maxS", "trackbars", xS);
    cv::setTrackbarPos( "minV", "trackbars", nV);
    cv::setTrackbarPos( "maxV", "trackbars", xV); 
    cv::setTrackbarPos( "di_er_kernel", "trackbars", dir); 
}

void setupTrackersValues(int gau, int iH, int xH, int nS, int xS, int nV, int xV, int dir){
    gaussian_kernel_size = gau;
    minH = iH;
    minS = nS;
    minV = nV;
    maxH = xH;
    maxS = xS;
    maxV = xV;
    di_er_kernel = dir;
}
#endif

void updateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr& msg){
    if (msg->id == 1 || msg->id == ALL_NODES ){  //TODO: change 
        // if input gaussian kernel size is possitive and 
        gaussian_kernel_size = (msg->gaussian_kernel_size > 0 && msg->gaussian_kernel_size% 2 != 0) ? msg->gaussian_kernel_size : gaussian_kernel_size + 1 ;
        minH = msg->minH;
        maxH = msg->maxH;
        minS = msg->minS;
        maxS = msg->maxS;  
        minV = msg->minV;
        maxV = msg->maxV;
        di_er_kernel = msg->di_er_kernel;
        if (di_er_kernel % 2 == 0 ) di_er_kernel = di_er_kernel > 0 ? di_er_kernel + 1 : 1;
        // ROS_INFO("Got new HSV values Min(%d, %d, %d), Max(%d, %d, %d) | di_er_kernel: %d, gaussian_kernel_size: %d", minH, minS, minV, maxH, maxS, maxV, di_er_kernel, gaussian_kernel_size);
    }
}

void hsvDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles){
    cv::Mat mask, bitwise_mask, hsvImg, cntImg, tmpImg = img.clone(), initImg = img.clone();
    cv::GaussianBlur(tmpImg, tmpImg, cv::Size(gaussian_kernel_size, gaussian_kernel_size), 5, 0);
    cv::cvtColor(tmpImg, hsvImg, cv::COLOR_BGR2HSV);
    cv::inRange(hsvImg, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);

    cv::Mat kernelDiER = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(di_er_kernel, di_er_kernel));
    cv::dilate(mask, mask, kernelDiER);
    cv::erode(mask, mask, kernelDiER);

    cv::Mat Points;
    cv::findNonZero(mask,Points);
    cv::Rect ballBound = boundingRect(Points);

// ------------------------------------------------------------------
//  Contours:
//     cntImg = mask.clone();
//     std::vector<cv::Vec4i> hierarchy;
//     std::vector<std::vector<cv::Point>> contours;
//     contours.clear();
//     std::vector<std::vector<cv::Point>> largest_contour;
//     double largest_area = 0;
//     cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//     for(size_t i = 0; i < contours.size(); i++){
//         if (cv::contourArea(contours[i]) > COUNTOURS_AREA_THRESHOLD){
// #ifdef TESTING_HSV
//         std::cout << i << ") Area: "  << cv::contourArea(contours[i]) << std::endl;
// #endif
//             if(cv::contourArea(contours[i]) > largest_area){
//                 largest_area = cv::contourArea(contours[i]);
//                 largest_contour.clear(); 
//                 largest_contour.push_back(contours[i]);
//             }
//         }
 
        
//     }

// #ifdef COUNTOURS_OBJ_AREA_THRESHOLD  
//     if(largest_area>=COUNTOURS_OBJ_AREA_THRESHOLD){ 
//         cv::drawContours(cntImg, largest_contour, -1, cv::Scalar(255, 0, 0), 2, cv::LINE_8, hierarchy, 0);
//         std::cout << "Ball exist" << std::endl;
//     }
// #else
//     cv::drawContours(cntImg, contours, -1, cv::Scalar(255, 0, 0), 2, cv::LINE_8, hierarchy, 0); 
// #endif
// -----------------------------------------------------------------


#ifdef TESTING_HSV
    // cv::imshow("Blur", tmpImg);
    // cv::imshow("HSV", hsvImg);
    // cv::imshow("Mask", mask);
    // cv::imshow("Contours", cntImg);
#endif

    cv::Point ballCenter = (ballBound.br() + ballBound.tl())*0.5;
    int radius = ballBound.height > ballBound.width ? ballBound.height/2 : ballBound.width/2;
    cv::Vec3f circle(ballCenter.x, ballCenter.y, radius);
    // ROS_INFO("New Circle: (%d, %d - %d)", ballCenter.x, ballCenter.y, radius);
    circles.push_back(circle);

#ifdef FREQUENCY
    // Frequency analysis
    cv::Rect leds;
    fa.update(initImg, hsvImg, ballBound, leds);
#endif

// ----------------------------------------------------------------------------------------------------------
#ifdef DEBUG_DISPLAY_TO_FRAME_INTERNAL_FRAMES
    // See bitwise image
    if(mask.type() == CV_8UC1){
        std::vector<cv::Mat> copies{mask, mask, mask};  
        cv::merge(copies,mask);
    }
    cv::bitwise_and(img, mask, bitwise_mask);
    float picScale = 0.24;
    int bottom = img.rows - hsvImg.rows*picScale - 10;
    copyImageTo(img, hsvImg, "HSV colorspace", 10, bottom, picScale);
    copyImageTo(img, mask, "Mask", 327, bottom, picScale);
    copyImageTo(img, bitwise_mask, "Bitwise Mask", 644, bottom, picScale);

#ifdef DEBUG_DISPLAY_TO_FRAME_HISTOGRAM
    cv::Mat rgbHistogram, hsvHistogram;
    getHistogram(initImg, rgbHistogram);
    getHistogram(hsvImg, hsvHistogram);

    cv::rectangle(img, leds,  cv::Scalar(0, 255, 0),2);
    
    copyImageTo(img, rgbHistogram, "RGB Histogram", 1014, 10, picScale);
    copyImageTo(img, hsvHistogram, "HSV Histogram", 1147, 10, picScale);
#endif
#endif
    // // Resize bitwise mask

    // calculateSensorSize(2*radius);
// ----------------------------------------------------------------------------------------------------------
    
}
