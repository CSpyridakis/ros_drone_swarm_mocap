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
#include "ros_drone_swarm_mocap/commands_from_master.h"

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
#include "worker/frequency.hpp"

image_transport::Publisher processedImage;
ros::Publisher processedData;
ros_drone_swarm_mocap::mocap_worker_data procData;

static bool send_dummy = false;
void addDummyDataForRanges(ros_drone_swarm_mocap::mocap_worker_data &procData){
    static int numOfDistances = 13;
    static float node_distances[] = {1.927, 1.343, 1.198, 2.333, 2.396, 2.162, 1.970, 1.672, 1.528, 1.611, 1.886, 2.110, 2.273};
    static int currentIndex = 0;
    static int repeatTimes = 0;

    if(send_dummy){
        procData.balls[0].distance_from_camera = node_distances[currentIndex];
        repeatTimes++;
        if(repeatTimes >= 30){
            repeatTimes = 0;
            currentIndex = (currentIndex + 1 <= numOfDistances) ? currentIndex + 1 : 0;  
        }
            
    }
}

void syncDataWithWorkersLocation(ros_drone_swarm_mocap::mocap_worker_data &procData){
    procData.pose.x = -1.0;
    procData.pose.y = 1.0;
    procData.pose.z = 1.36;

    procData.pose.roll = 0;
    procData.pose.pitch = 0;
    procData.pose.yaw = 0;
}

//  ==============================================================================================================
//  ==============================================================================================================
//  ==============================================================================================================

ros_drone_swarm_mocap::master_time masterPgk;

void getCommandsFromMasterCB(const ros_drone_swarm_mocap::commands_from_master::ConstPtr& commands){
    ROS_INFO("Receive commands from master");
    // send_dummy = cmds.workers_enable_messages;
    send_dummy = commands->workers_send_dummy;
}

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
    procData.header.stamp = masterPgk.header.stamp;
    syncDataWithWorkersLocation(procData);

    // TODO: Remove ONLY for sync testing!
    addDummyDataForRanges(procData);

    // Publish processed data to topics
    sensor_msgs::ImagePtr procImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", feedOut).toImageMsg();
    processedImage.publish(procImage);
    processedData.publish(procData);
}

void sendDummy(){
    procData.balls.clear();
    ros_drone_swarm_mocap::detected_ball_data bd;
    procData.balls.push_back(bd);

    procData.master_data = masterPgk;
    syncDataWithWorkersLocation(procData);

    // TODO: Remove ONLY for sync testing!
    addDummyDataForRanges(procData);

    // Publish processed data to topics
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
    procData.camera.imageHeightInPixels                 = camera_parameters->height;
    procData.camera.imageWidthInPixels                  = camera_parameters->width;
    procData.camera.XfocalLengthInMillimeters           = camera_parameters->P[0];
    procData.camera.YfocalLengthInMillimeters           = camera_parameters->P[5];
    procData.camera.XFieldOfViewInAngles                = 2 * atan((float)procData.camera.imageWidthInPixels  / (2 * (float)procData.camera.XfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.YFieldOfViewInAngles                = 2 * atan((float)procData.camera.imageHeightInPixels / (2 * (float)procData.camera.YfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.objectsRealSizeInMeter              = objRealSize;
    procData.camera.XsensorSizeInMillimeters            = XsensorSizeInMillimeter;
    procData.camera.YsensorSizeInMillimeters            = YsensorSizeInMillimeter;

    // Sensor's init positions
    procData.pose.x                                     = node_x;
    procData.pose.y                                     = node_y;
    procData.pose.z                                     = node_z;
    procData.pose.roll                                  = node_roll;
    procData.pose.pitch                                 = node_pitch;
    procData.pose.yaw                                   = node_yaw;

    // Create topics names
    std::string subImageTopic                           = "/usb_cam_" + std::to_string(nodeID) + "/image_color";
    std::string pubImageTopic                           = "/usb_cam_" + std::to_string(nodeID) + "/processed_image";
    std::string pubImageDataTopic                       = "/node/" + std::to_string(nodeID) + "/processed_data";

    // Create actual subscribers and publishers 
    image_transport::Subscriber rawImage                = it.subscribe(subImageTopic, 1, CallbackFunction);
    processedImage                                      = it.advertise(pubImageTopic, 1);
    processedData                                       = n.advertise<ros_drone_swarm_mocap::mocap_worker_data>(pubImageDataTopic, 30);
    ros::Subscriber master_time                         = n.subscribe("/master/time", 5, updateMasterTimeCB); 
    ros::Subscriber master_commands                     = n.subscribe("/master/commands", 5, getCommandsFromMasterCB);

    // Just for debugging, create subscribers to change detection parameters through topics 
#ifdef DEBUG
    ros::Subscriber hue_sub                             = n.subscribe("/node/" + std::to_string(nodeID) + "/hsv_params", 5, updateHSVvaluesCallback); 
    ros::Subscriber hough_sub                           = n.subscribe("/node/" + std::to_string(nodeID) + "/hough_params",5, updateHoughvaluesCallback);
    ros::Subscriber freq_hue_sub                        = n.subscribe("/node/" + std::to_string(nodeID) + "/frequency/hsv_params", 5, freqUpdateHSVvaluesCallback); 
#endif

    D_INIT();

    // TODO: REMOVE AFTER TESTING
    ros::Rate loop_rate(10);  // Number of messages/sec
    while(ros::ok()){
        sendDummy();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}