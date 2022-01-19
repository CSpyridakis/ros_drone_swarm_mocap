#include <ros/ros.h>
#include <iostream>
#include <string> 
#include <time.h>

#include "station/trilateration.hpp"
#include "ros_drone_swarm_mocap/master_time.h"

int packet_id = 0;

int main(int argc, char **argv){
    ros::init(argc, argv, "ball_localization");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<ros_drone_swarm_mocap::master_time>("/master/time", 5);
    ros_drone_swarm_mocap::master_time msg;
    ros::Rate loop_rate(2);  // Number of messages/sec

    while(ros::ok()){
        msg.packet_id = packet_id ++;
        msg.internal_time = clock();
        msg.ros_time = ros::Time::now();
        
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}