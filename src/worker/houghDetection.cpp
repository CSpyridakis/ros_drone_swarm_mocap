#include "worker/houghDetection.hpp"
#include "worker/misc.hpp"
#define ALL_NODES 0

static int accumulator_resolution = 2;
static int median_blur = 5;
static int di_er_kernel = 1;
static int min_distance_between_circles = 500;
static int canny_high_threshold = 120;
static int threshold_for_center_detection = 100;  

void updateHoughvaluesCallback(const ros_drone_swarm_mocap::hough_values::ConstPtr& msg){
    if(msg->id == 1 || msg->id == ALL_NODES){   //TODO: change
        accumulator_resolution = msg->acc_res;

        median_blur = msg->median_blur;
        if(median_blur % 2 == 0) median_blur = median_blur > 0 ? median_blur + 1 : 1;

        di_er_kernel = msg->di_er_kernel;
        if (di_er_kernel % 2 == 0 ) di_er_kernel = di_er_kernel > 0 ? di_er_kernel + 1 : 1;

        min_distance_between_circles = msg->min_dist_betw_cir ;
        canny_high_threshold = msg->canny_high_res ;
        threshold_for_center_detection = msg->threshold_for_center ;
        ROS_INFO("Got new Hough values acc:[%d] median: [%d] min_dist:[%d] canny:[%d] thres:[%d] dil/er:[%d]", accumulator_resolution, median_blur, min_distance_between_circles, canny_high_threshold, threshold_for_center_detection, di_er_kernel);
    }
}

void houghDetection(cv::Mat &img, std::vector<cv::Vec3f> &circles){
    cv::Mat imgBlur, imgCanny, imgHough, cannyTMP;
    cv::cvtColor(img, imgBlur, cv::COLOR_BGR2GRAY); 
    cv::medianBlur(imgBlur, imgBlur, median_blur);
    
    cv::Canny(imgBlur, imgCanny, canny_high_threshold, canny_high_threshold/2);

    cv::Mat kernelDiER = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(di_er_kernel, di_er_kernel));
    cv::dilate(imgCanny, imgCanny, kernelDiER);
    cv::erode(imgCanny, imgCanny, kernelDiER);

    imgHough = imgCanny.clone();

    cv::Canny(imgCanny, cannyTMP, canny_high_threshold, canny_high_threshold/2);

    cv::HoughCircles(imgHough, circles, cv::HOUGH_GRADIENT,
                accumulator_resolution,         // Accumulator resolution 
                min_distance_between_circles,   // Min distance between circles
                canny_high_threshold,           // Canny high threshold
                threshold_for_center_detection, // Threshold for center detection
                0, 200);                        // Min and max radius

    
    copyImageTo(img, imgBlur, "Gray Image Blur", 10, 10, 0.3);
    copyImageTo(img, cannyTMP, "Canny org", imgCanny.cols*0.3 +  20, 10, 0.3);
    copyImageTo(img, imgCanny, "Canny edge detector", img.cols - imgCanny.cols*0.3 - 10, 10, 0.3);
}