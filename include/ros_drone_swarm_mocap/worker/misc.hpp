#ifndef MISC_HPP
#define MISC_HPP

#include <opencv2/opencv.hpp>

#define DEBUG_ALL
#ifdef DEBUG_ALL
    #define DEBUG_EXTR_DATA_TO_FILE
    #define DEBUG_SAVE_HW_PERFORMANCE_TO_FILES
    #define DEBUG_DISPLAY_TO_FRAME_INTERNAL_FRAMES
    #define DEBUG_DISPLAY_TO_FRAME_HISTOGRAM
#endif

/**
 * \brief ROS image transfer needs CV_8UC3 to display image in web server, so fix Mat in other case
 * 
 * \param img
 */
void fixMatForImageTransfer(cv::Mat &img);

/**
 * \brief
 * 
 * \param img1
 * \param img2
 * \param outimage
 */
void combineImages(cv::Mat img1, cv::Mat img2, cv::Mat &outimage);


void copyImageTo(cv::Mat &img, const cv::Mat toCopyImg, const std::string text, const int left_offset, const int bottom_offset, const float scale);


void getHistogram(const cv::Mat img, cv::Mat &histogram);

#endif //MISC_HPP
