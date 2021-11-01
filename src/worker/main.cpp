#include <ros/ros.h>
#include <iostream>
#include <string> 

// Messages
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

// Opencv and image transport
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Detection
#include "worker/ballDetection.hpp"
#include "worker/houghDetection.hpp"
#include "worker/hsvDetection.hpp"


image_transport::Publisher processedImage;
ros::Publisher processedData;
ros_drone_swarm_mocap::mocap_worker_data procData;



//  ==============================================================================================================
//  ==============================================================================================================
//  ==============================================================================================================



void CallbackFunction(const sensor_msgs::Image::ConstPtr& inImage){
    cv::Mat feedOut;
    cv::Mat feedIn;
    
    // Convert frame from ROS message to OpenCV Mat
    try{
        feedIn = cv_bridge::toCvCopy(inImage, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("CV_BRIDGE COPY EXPEPTION");
        return;
    }
    if (feedIn.empty()) return;
    
    // Detect Ball
    detectBall(feedIn, feedOut, procData);

    // Publish processed data to topics
    sensor_msgs::ImagePtr procImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", feedOut).toImageMsg();
    processedImage.publish(procImage);
    processedData.publish(procData);
}



//  ==============================================================================================================
//  ==============================================================================================================
//  ==============================================================================================================



int main(int argc, char **argv){
    ros::init(argc, argv, "detect_ball");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Read parameters and other important values
    int nodeID = 3;
    float objRealSize = 0.0;
    float XsensorSizeInMillimeter = 0.0;
    float YsensorSizeInMillimeter = 0.0;
    try{
        n.getParam("nodeID", nodeID);
        n.getParam("ballRealSizeInMeter", objRealSize);
        n.getParam("XsensorSizeInMillimeter", XsensorSizeInMillimeter);
        n.getParam("YsensorSizeInMillimeter", YsensorSizeInMillimeter);
        ROS_INFO("Parameters loaded ID[%d]", nodeID);
    }
    catch(int e){
        ROS_WARN("Parameters load failed");
    }
    procData.nodeID = nodeID;
    sensor_msgs::CameraInfo::ConstPtr camera_parameters = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/usb_cam_" + std::to_string(nodeID) + "/camera_info", n);
    procData.imageHeightInPixels = camera_parameters->height;
    procData.imageWidthInPixels = camera_parameters->width;
    procData.XfocalLengthInMillimeters = camera_parameters->P[0];
    procData.YfocalLengthInMillimeters = camera_parameters->P[5];
    procData.XFieldOfViewInAngles = 2 * atan((float)procData.imageWidthInPixels  / (2 * (float)procData.XfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.YFieldOfViewInAngles = 2 * atan((float)procData.imageHeightInPixels / (2 * (float)procData.YfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.objectsRealSizeInMeter = objRealSize;
    procData.XsensorSizeInMillimeters = XsensorSizeInMillimeter;
    procData.YsensorSizeInMillimeters = YsensorSizeInMillimeter;

    // Create topics names
    std::string subImageTopic = "/usb_cam_" + std::to_string(nodeID) + "/image_color";
    std::string pubImageTopic = "/usb_cam_" + std::to_string(nodeID) + "/processed_image";
    std::string pubImageDataTopic = "/node/" + std::to_string(nodeID) + "/processed_data";

    // Create actual subscribers and publishers 
    image_transport::Subscriber rawImage = it.subscribe(subImageTopic, 1, CallbackFunction);
    processedImage = it.advertise(pubImageTopic, 1);
    processedData = n.advertise<ros_drone_swarm_mocap::mocap_worker_data>(pubImageDataTopic, 30);

    // Just for debugging, create subscribers to change detection parameters through topics 
#ifdef DEBUG
    ros::Subscriber hue_sub = n.subscribe("/node/" + std::to_string(nodeID) + "/hsv_params", 5, updateHSVvaluesCallback); 
    ros::Subscriber hough_sub = n.subscribe("/node/" + std::to_string(nodeID) + "/hough_params",5, updateHoughvaluesCallback);
#endif

    ros::spin();
    return 0;
}