#include <ros/ros.h>
#include <iostream>
#include <string> 

// Messages
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/camera_params.h"
#include "ros_drone_swarm_mocap/obj_pose.h"
#include "ros_drone_swarm_mocap/master_time.h"

// Opencv and image transport
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Detection
#include "worker/ballDetection.hpp"
#include "worker/houghDetection.hpp"
#include "worker/hsvDetection.hpp"

#include "worker/misc.hpp"

#include "statistics/performance.hpp"

image_transport::Publisher processedImage;
ros::Publisher processedData;
ros_drone_swarm_mocap::mocap_worker_data procData;



//  ==============================================================================================================
//  ==============================================================================================================
//  ==============================================================================================================

ros_drone_swarm_mocap::master_time masterPgk;

void updateMasterTimeCB(const ros_drone_swarm_mocap::master_time::ConstPtr& timePkg){
    masterPgk = *timePkg;
}

void CallbackFunction(const sensor_msgs::Image::ConstPtr& inImage){
    cv::Mat feedOut;
    cv::Mat feedIn;
    
    // Convert frame from ROS message to OpenCV Mat
    try{
        feedIn = cv_bridge::toCvCopy(inImage, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("CV_BRIDGE COPY EXCEPTION");
        return;
    }
    if (feedIn.empty()) return;
    
    // Detect Ball
    detectBall(feedIn, feedOut, procData);

    procData.master_data = masterPgk;

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
    float node_x = 0.0, node_y = 0.0, node_z = 0.0, node_roll = 0.0, node_pitch = 0.0, node_yaw = 0.0; 
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
    procData.camera.imageHeightInPixels = camera_parameters->height;
    procData.camera.imageWidthInPixels = camera_parameters->width;
    procData.camera.XfocalLengthInMillimeters = camera_parameters->P[0];
    procData.camera.YfocalLengthInMillimeters = camera_parameters->P[5];
    procData.camera.XFieldOfViewInAngles = 2 * atan((float)procData.camera.imageWidthInPixels  / (2 * (float)procData.camera.XfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.YFieldOfViewInAngles = 2 * atan((float)procData.camera.imageHeightInPixels / (2 * (float)procData.camera.YfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.objectsRealSizeInMeter = objRealSize;
    procData.camera.XsensorSizeInMillimeters = XsensorSizeInMillimeter;
    procData.camera.YsensorSizeInMillimeters = YsensorSizeInMillimeter;

    // Sensor's init positions
    procData.pose.x = node_x;
    procData.pose.y = node_y;
    procData.pose.z = node_z;
    procData.pose.roll = node_roll;
    procData.pose.pitch = node_pitch;
    procData.pose.yaw = node_yaw;

    // Create topics names
    std::string subImageTopic = "/usb_cam_" + std::to_string(nodeID) + "/image_color";
    std::string pubImageTopic = "/usb_cam_" + std::to_string(nodeID) + "/processed_image";
    std::string pubImageDataTopic = "/node/" + std::to_string(nodeID) + "/processed_data";

    // Create actual subscribers and publishers 
    image_transport::Subscriber rawImage = it.subscribe(subImageTopic, 1, CallbackFunction);
    processedImage = it.advertise(pubImageTopic, 1);
    processedData = n.advertise<ros_drone_swarm_mocap::mocap_worker_data>(pubImageDataTopic, 30);
    ros::Subscriber master_time = n.subscribe("/master/time", 5, updateMasterTimeCB); 

    // Just for debugging, create subscribers to change detection parameters through topics 
#ifdef DEBUG
    ros::Subscriber hue_sub = n.subscribe("/node/" + std::to_string(nodeID) + "/hsv_params", 5, updateHSVvaluesCallback); 
    ros::Subscriber hough_sub = n.subscribe("/node/" + std::to_string(nodeID) + "/hough_params",5, updateHoughvaluesCallback);
#endif

    D_INIT();

    ros::spin();
    return 0;
}