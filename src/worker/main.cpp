#include <ros/ros.h>
#include <iostream>
#include <string> 

#include "sensor_msgs/Image.h"
#include "ros_drone_swarm_mocap/mocap_worker_data.h"

#include "worker/ballDetection.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

int nodeID = 0;

image_transport::Publisher processedImage;
ros::Publisher processedData;

void CallbackFunction(const sensor_msgs::Image::ConstPtr& inImage){
    sensor_msgs::ImagePtr procImage;
    ros_drone_swarm_mocap::mocap_worker_data procData;
    procData.nodeID = nodeID;

    cv::Mat feedOut;
    cv::Mat feedIn = cv_bridge::toCvCopy(inImage)->image;
    if (feedIn.empty()) return;
    
    detectBall(feedIn, feedOut, procData);

    procImage = cv_bridge::CvImage(std_msgs::Header(), "rgb8", feedOut).toImageMsg();
    processedImage.publish(procImage);
    processedData.publish(procData);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "detect_ball");
    ros::NodeHandle n;
    
    try{
        n.getParam("nodeID", nodeID);
        ROS_INFO("Parameters loaded");
    }
    catch(int e){
        ROS_WARN("Parameters load failed");
    }

    std::string subImageTopic = "/usb_cam_" + std::to_string(nodeID) + "/image_raw";
    std::string pubImageTopic = "/usb_cam_" + std::to_string(nodeID) + "/processed_image";
    std::string pubImageDataTopic = "/node/" + std::to_string(nodeID) + "/processed_data";

    image_transport::ImageTransport it(n);

    image_transport::Subscriber rawImage = it.subscribe(subImageTopic, 1, CallbackFunction);
    processedImage = it.advertise(pubImageTopic, 1);

    processedData = n.advertise<ros_drone_swarm_mocap::mocap_worker_data>(pubImageDataTopic, 30);

    ros::spin();
    return 0;
}