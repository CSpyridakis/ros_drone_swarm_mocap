#include <ros/ros.h>
#include <iostream>
#include <string> 

#include "station/trilateration.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "ball_localization");
    ros::NodeHandle n;

    ros::spin();
    return 0;
}