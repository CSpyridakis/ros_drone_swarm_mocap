#include <ros/ros.h>
#include <iostream>
#include <string> 
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "station/trilateration.hpp"
#include "ros_drone_swarm_mocap/master_time.h"

#include "ros_drone_swarm_mocap/mocap_worker_data.h"

using namespace ros_drone_swarm_mocap;
using namespace message_filters;

void localizeObjectCB(const ros_drone_swarm_mocap::mocap_worker_data::ConstPtr& procData1, 
                      const ros_drone_swarm_mocap::mocap_worker_data::ConstPtr& procData2, 
                      const ros_drone_swarm_mocap::mocap_worker_data::ConstPtr& procData3, 
                      const ros_drone_swarm_mocap::mocap_worker_data::ConstPtr& procData4){

    ros_drone_swarm_mocap::mocap_worker_data pd[] = {(*procData1),  (*procData2), (*procData3), (*procData4)};

    std::vector<anchor_data> anchors;
    Point objectsPose;
    
    for(int i=0;i<4;i++){
        anchor_data anc;

        anc.pose = {{pd[i].pose.x, pd[i].pose.y, pd[i].pose.z},{pd[i].pose.roll, pd[i].pose.pitch, pd[i].pose.yaw}};
        anc.objectsDistance = pd[i].balls[0].distance_from_camera + BALL_RADIOUS;

        anchors.push_back(anc);
        // ROS_INFO("N%d - [%f, %f, %f] - %f", pd[i].nodeID, anc.pose.pos.x, anc.pose.pos.y, anc.pose.pos.z, anc.objectsDistance);
    }
            
    trilateration(anchors, objectsPose);

    ROS_INFO("B (%f, %f, %f)\n", objectsPose.pos.x, objectsPose.pos.y, objectsPose.pos.z);
}

int packet_id = 0;
int main(int argc, char **argv){
    ros::init(argc, argv, "ball_localization");
    ros::NodeHandle n;

    message_filters::Subscriber<ros_drone_swarm_mocap::mocap_worker_data> worker_1(n, "/node/1/processed_data", 1);
    message_filters::Subscriber<ros_drone_swarm_mocap::mocap_worker_data> worker_2(n, "/node/2/processed_data", 1);
    message_filters::Subscriber<ros_drone_swarm_mocap::mocap_worker_data> worker_3(n, "/node/3/processed_data", 1);
    message_filters::Subscriber<ros_drone_swarm_mocap::mocap_worker_data> worker_4(n, "/node/4/processed_data", 1);

  typedef message_filters::sync_policies::ApproximateTime<ros_drone_swarm_mocap::mocap_worker_data,
                                                            ros_drone_swarm_mocap::mocap_worker_data,
                                                            ros_drone_swarm_mocap::mocap_worker_data,
                                                            ros_drone_swarm_mocap::mocap_worker_data> sync_pol; 

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), worker_1, worker_2, worker_3, worker_4);
    sync.registerCallback(boost::bind(&localizeObjectCB,_1,_2,_3,_4));

    // -----------------
    ros::Publisher chatter_pub = n.advertise<ros_drone_swarm_mocap::master_time>("/master/time", 5);
    ros_drone_swarm_mocap::master_time msg;
    ros::Rate loop_rate(2);  // Number of messages/sec

    while(ros::ok()){
        msg.packet_id = packet_id ++;
        msg.internal_time = clock();
        msg.ros_time = ros::Time::now();
        msg.header.stamp = ros::Time::now();
        
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}