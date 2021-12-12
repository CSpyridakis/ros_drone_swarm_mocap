#ifndef MISC_HPP
#define MISC_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"


#define DEBUG_FUNCTIONS
#define DEBUG_IMAGE_PRINTS

#ifdef DEBUG_FUNCTIONS
#include <sys/types.h>
#include <sys/sysinfo.h>
#include <filesystem>
#endif

static std::string folder = "/home/cs-du/catkin_ws/src/ros_drone_swarm_mocap/test/experiments/";
static ros::Time initTime;
static struct sysinfo system_info;

#define D_INIT() {  initTime = ros::Time::now(); \
                    std::ofstream myfile1(folder + "time.txt");  myfile1.close(); \
                    std::ofstream myfile2(folder + "cpu.txt");  myfile2.close(); \
                    std::ofstream myfile3(folder + "ram.txt");  myfile3.close(); \
                    std::ofstream myfile4(folder + "distance.txt");  myfile4.close();\
                    std::ofstream myfile5(folder + "angles.txt");  myfile5.close(); \
                    std::ofstream myfile6(folder + "power.txt");  myfile6.close();\
                    std::filesystem::remove_all(folder + "images/"); \
                    namespace fs = std::filesystem;
                    fs::create_directories(folder + "images");} 

#define D_TIME(f, text) { std::ofstream myfile(folder + "time.txt", std::ios_base::app); \
                    ros::Time beforeTime = ros::Time::now(); \
                    f; \
                    ros::Time afterTime = ros::Time::now(); \
                    myfile << (ros::Time::now().toSec()) << " : " << text << (afterTime - beforeTime) << std::endl; \
                    myfile.close(); }

#define D_CPU() {std::ofstream myfile(folder + "cpu.txt", std::ios_base::app); \
                    int cpu_usage = 0 ; \
                    myfile << "[" << (ros::Time::now().toSec()) << " : " << cpu_usage << std::endl; \
                    myfile.close();}

#define D_RAM() {std::ofstream myfile(folder + "ram.txt", std::ios_base::app); \
                    sysinfo(&system_info); \
                    myfile << (ros::Time::now().toSec()) << " : " << (system_info.totalram - system_info.freeram) << " / " << system_info.totalram << std::endl; \
                    myfile.close();}

#define D_POWER() { std::ofstream myfile(folder + "power.txt", std::ios_base::app); \
                myfile << (ros::Time::now().toSec()) << " : " << n << " -x: " << x << " y: " << y << std::endl; \
                myfile.close();}

#define D_DISTANCE(n, distance) { std::ofstream myfile(folder + "distance.txt", std::ios_base::app); \
                                    myfile << (ros::Time::now().toSec()) << " : " << n << " - " << distance << std::endl; \
                                    myfile.close();}

#define D_ANGLES(n, x, y) { std::ofstream myfile(folder + "angles.txt", std::ios_base::app); \
                                    myfile << (ros::Time::now().toSec()) << " : " << n << " -x: " << x << " y: " << y << std::endl; \
                                    myfile.close();}

#define DEBUG_DA(n, distance, anglex, angley){  D_DISTANCE(n, distance) \
                                                D_ANGLES(n, anglex, angley)}

#define SAVE_FRAME(frame){  std::string filename = folder + "images/experiment" + std::to_string(ros::Time::now().toSec()) + ".png"; \
                            cv::imwrite(filename, frame);}
                                                

/**
 * 
 */
float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis);

/**
 */
float calculateSensorsSizeFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float objectsDistanceFromCameraInMeters);

/**
 */
float calculateSensorSize(int objectSizeInPixels, float objectsDistanceFromCameraInMeters, 
                        ros_drone_swarm_mocap::mocap_worker_data& procData);

/**
 * \brief
 * 
 * \param focalLengthInMillimeter
 * \param objectsRealSizeInMeter
 * \param imageSizeInPixels
 * \param objectsSizeInPixels
 * \param sensorSizeInMillimeter
 * 
 * \return 
 */
float calculateDistanceFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter );


/**
 * \brief
 * 
 * \param objectSizeInPixels
 * \param procData
 * 
 * \return 
 */
float calculateDistanceWithDataStruct(int objectSizeInPixels, ros_drone_swarm_mocap::mocap_worker_data& procData);


/**
 * \brief
 * 
 * \param circles
 * \param procData
 */
void saveDistancesToProcData(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData);



/**
 * \brief ROS image transfer needs CV_8UC3 to display image in web server, so fix Mat in other case
 * 
 * \param img
 */
void fixMatForImageTransfer(cv::Mat &img);

/**
 * \brief
 * 
 * \param img1
 * \param img2
 * \param outimage
 */
void combineImages(cv::Mat img1, cv::Mat img2, cv::Mat &outimage);


void copyImageTo(cv::Mat &img, const cv::Mat toCopyImg, const std::string text, const int left_offset, const int bottom_offset, const float scale);

#endif //MISC_HPP