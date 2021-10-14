#include <ros/ros.h>
#include <iostream>
#include <string> 

// #include "ros_drone_swarm_mocap/mocap_client_data.h"

// ros::Publisher chatter_pub;
// ros::Subscriber sub;

// void CallbackFunction(const std_msgs::String::ConstPtr& msg){
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "pub");
    ros::NodeHandle n;
    
    // int nodeID = 0;
    // try{
    //     n.getParam("nodeID", nodeID);
    //     ROS_INFO("Parameters loaded");
    // }
    // catch(int e){
    //     ROS_WARN("Parameters load failed");
    // }

    // // chatter_pub = n.advertise<std_msgs::String>("/ros_example/basic_comm", 1000);

    // std::string subTopic = "/usb_cam_" + std::to_string(nodeID) + "/image_raw";
    // sub = n.subscribe(subTopic, 1000, CallbackFunction);
    // ros::spin();
    return 0;
}