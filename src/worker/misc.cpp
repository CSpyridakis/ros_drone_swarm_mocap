#include "worker/misc.hpp"

float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis = false){
    float angle = (float)( (std::abs( (float)pixelLenght/2 - NthPixel) * fov ) / pixelLenght);  ///< Calculate angle

    if (yaxis) return (float)(NthPixel >= pixelLenght/2 ) ? -angle : angle  ;       // Calculate sign (for y axis ymiddle < 0 -> positive angles)
    return (float)(NthPixel >= pixelLenght/2 ) ? angle : -angle  ;                  // Calculate sign (for x axis xmiddle < 0 -> negative angles)
}

float calculateSensorsSizeFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float objectsDistanceFromCameraInMeters){
    
    // ROS_INFO("\nfocalLengthInMillimeter: %f  \nobjectsRealSizeInMeter: %f \nimageSizeInPixels: %d \nobjectsSizeInPixels: %d \nobjectsDistanceFromCameraInMeters:%f\n", focalLengthInMillimeter, objectsRealSizeInMeter, imageSizeInPixels, objectsSizeInPixels, objectsDistanceFromCameraInMeters);

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / ((float)objectsSizeInPixels * objectsDistanceFromCameraInMeters) );
}

float calculateSensorSize(int objectSizeInPixels, float objectsDistanceFromCameraInMeters, 
                        ros_drone_swarm_mocap::mocap_worker_data& procData){
    return calculateSensorsSizeFull(procData.XfocalLengthInMillimeters, procData.objectsRealSizeInMeter, procData.imageWidthInPixels,
                            objectSizeInPixels, objectsDistanceFromCameraInMeters);
}

float calculateDistanceFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter ){

    // ROS_INFO("\nfocalLengthInMillimeter: %f  \nobjectsRealSizeInMeter: %f \nimageSizeInPixels: %d \nobjectsSizeInPixels: %d \nsensorSizeInMillimeter:%f\n", focalLengthInMillimeter, objectsRealSizeInMeter, imageSizeInPixels, objectsSizeInPixels, sensorSizeInMillileter);

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / ((float)objectsSizeInPixels * sensorSizeInMillileter) );
}


float calculateDistanceWithDataStruct(int objectSizeInPixels, ros_drone_swarm_mocap::mocap_worker_data& procData){
    return calculateDistanceFull(procData.XfocalLengthInMillimeters, procData.objectsRealSizeInMeter, 
                                    procData.imageWidthInPixels, objectSizeInPixels, procData.XsensorSizeInMillimeters);
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
        bd.xangle = calculateAngle(procData.XFieldOfViewInAngles, procData.imageWidthInPixels, bd.image_plane_x);
        bd.yangle = calculateAngle(procData.YFieldOfViewInAngles, procData.imageHeightInPixels, bd.image_plane_y, true);
        // ROS_INFO("Circle - %d Center: (%d, %d) | Distance: %f | Angle x: %f | Angle y: %f", k,  bd.image_plane_x, bd.image_plane_y,  bd.distance_from_camera,  bd.xangle, bd.yangle);
        // ROS_INFO("Sensorsize: %f\n", calculateSensorSize(2*bd.image_plane_r, 1.0, procData));
    }
    procData.balls.push_back(bd);
}


void copyImageTo(cv::Mat &img, const cv::Mat toCopyImg, const std::string text, const int left_offset = 10, const int bottom_offset = 10, const float scale = 0.3){
    int resC = int(toCopyImg.cols * scale);
    int resR = int(toCopyImg.rows * scale);

    cv::Mat tmpImg;
    cv::resize(toCopyImg, tmpImg, cv::Size(resC,resR), cv::INTER_LINEAR);

    fixMatForImageTransfer(img);
    fixMatForImageTransfer(tmpImg);

    cv::putText(tmpImg, text, cv::Point(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0,255,0), 1);
    int left_off = (left_offset + resC < img.cols) ? left_offset : left_offset * scale + 20;
    int bottom_off = (bottom_offset + resR < img.rows) ? img.rows - tmpImg.rows - 10 : bottom_offset * scale + 20;
    // ROS_INFO("[%d, %d] -> [%d, %d , %d, %d]", toCopyImg.cols, toCopyImg.rows, left_off, bottom_off, resC, resR);
    tmpImg.copyTo(img(cv::Rect(left_off, bottom_off, resC, resR)));
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