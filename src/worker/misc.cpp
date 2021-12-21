#include <opencv2/core/fast_math.hpp>

#include "worker/misc.hpp"
#include "distance-angle/distance-angle.hpp"
#include "statistics/performance.hpp"

float calculateSensorSize(  int objectSizeInPixels, 
                            float objectsDistanceFromCameraInMeters, 
                            ros_drone_swarm_mocap::mocap_worker_data& procData){

    return calculateSensorsSizeFull(procData.camera.XfocalLengthInMillimeters, 
                                    procData.camera.objectsRealSizeInMeter, 
                                    procData.camera.imageWidthInPixels,
                                    objectSizeInPixels, 
                                    objectsDistanceFromCameraInMeters);
}

float calculateDistanceWithDataStruct(  int objectSizeInPixels, 
                                        ros_drone_swarm_mocap::mocap_worker_data& procData){

    return calculateDistanceFull(   procData.camera.XfocalLengthInMillimeters, 
                                    procData.camera.objectsRealSizeInMeter, 
                                    procData.camera.imageWidthInPixels, 
                                    objectSizeInPixels, 
                                    procData.camera.XsensorSizeInMillimeters);
}

void fixMatForImageTransfer(cv::Mat &img){
    if(img.type() == CV_8UC1){
        std::vector<cv::Mat> copies{img, img, img};  
        cv::merge(copies,img);
    }
}

void saveDistancesToProcData(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData){
    ros_drone_swarm_mocap::detected_ball_data bd;
    for( uint k = 0; k < circles.size(); k++ ){
        bd.image_plane_x = circles[k][0];
        bd.image_plane_y = circles[k][1];
        bd.image_plane_r = circles[k][2];
        bd.distance_from_camera = calculateDistanceWithDataStruct(2*bd.image_plane_r, procData);
        bd.xangle = calcucateAngleX(procData.camera.XFieldOfViewInAngles, procData.camera.imageWidthInPixels, bd.image_plane_x);
        bd.yangle = calcucateAngleY(procData.camera.YFieldOfViewInAngles, procData.camera.imageHeightInPixels, bd.image_plane_y);
        // ROS_INFO("Circle - %d Center: (%d, %d) | Distance: %f | Angle x: %f | Angle y: %f", k,  
                    //  bd.image_plane_x, bd.image_plane_y,  bd.distance_from_camera,  bd.xangle, bd.yangle);
        // ROS_INFO("Sensorsize: %f\n", calculateSensorSize(2*bd.image_plane_r, 1.0, procData));
#ifdef DEBUG_EXTR_DATA_TO_FILE
        DEBUG_DA(ros::Time::now().toSec(), k, bd.distance_from_camera, bd.xangle, bd.yangle);
#endif
    }
    procData.balls.push_back(bd);
}


void copyImageTo(cv::Mat &img, const cv::Mat toCopyImg, const std::string text, const int top_left_x = 10, const int top_left_y = 10, const float scale = 0.3){
    int resX = int(toCopyImg.cols * scale);
    int resY = int(toCopyImg.rows * scale);

    cv::Mat tmpImg;
    cv::resize(toCopyImg, tmpImg, cv::Size(resX,resY), cv::INTER_LINEAR);

    fixMatForImageTransfer(img);
    fixMatForImageTransfer(tmpImg);

    cv::putText(tmpImg, text, cv::Point(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0,255,0), 1);
    int tl_x = (top_left_x + resX > img.cols || top_left_x < 0) ? 10 : top_left_x;
    int tl_y = (top_left_y + resY > img.rows || top_left_y < 0) ? 10 : top_left_y;
    // ROS_INFO("{%s} (offL:%d, offT:%d) | [IX:%d, IY:%d] -> [tlx:%d, tly:%d , w:%d, h:%d]", text.c_str(), top_left_x, top_left_y, toCopyImg.cols, toCopyImg.rows, tl_x, tl_y, resX, resY);
    tmpImg.copyTo(img(cv::Rect(tl_x, tl_y, resX, resY)));
}


void combineImages(cv::Mat img1, cv::Mat img2, cv::Mat &outimage){
    fixMatForImageTransfer(img1);
    fixMatForImageTransfer(img2);

    int rows = img1.rows + img2.rows;
    int cols = std::max(img1.cols, img2.cols);

    cv::Mat res = cv::Mat(rows, cols, CV_8UC3);

    img1.copyTo(res(cv::Rect(0, 0, img1.cols, img1.rows)));
    img2.copyTo(res(cv::Rect(0, img1.rows, img2.cols, img2.rows)));
    outimage = res.clone();
}

#define _B 0
#define _G 1
#define _R 2

void getHistogram(const cv::Mat img, cv::Mat &histogram){
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;

    cv::Mat hist[3];
    std::vector<cv::Mat> channels(img.channels());
    cv::split(img, channels);
    
    cv::calcHist(&channels[_B], 1, 0, cv::Mat(), hist[_B], 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&channels[_G], 1, 0, cv::Mat(), hist[_G], 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&channels[_R], 1, 0, cv::Mat(), hist[_R], 1, &histSize, &histRange, uniform, accumulate);

    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound((double) hist_w/histSize );
    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));

    for(int i = 1; i<histSize; i++){
        cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(hist[_B].at<float>(i-1)) ),
            cv::Point(bin_w*(i), hist_h - cvRound(hist[_B].at<float>(i)) ),
            cv::Scalar(255, 0, 0), 2, 8, 0);
        cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(hist[_G].at<float>(i-1)) ),
            cv::Point(bin_w*(i), hist_h - cvRound(hist[_G].at<float>(i)) ),
            cv::Scalar(0, 255, 0), 2, 8, 0);
        cv::line (histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(hist[_R].at<float>(i-1)) ),
            cv::Point(bin_w*(i), hist_h - cvRound(hist[_R].at<float>(i))),
            cv::Scalar(0, 0, 255), 2, 8, 0);
    }
    histogram = histImage.clone();
}
