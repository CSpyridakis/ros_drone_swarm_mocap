#ifndef MISC_HPP
#define MISC_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "ros_drone_swarm_mocap/detected_ball_data.h"

#define DEBUG_FUNCTIONS
#define DEBUG_IMAGE_PRINTS

static std::string project_dir = "/home/cs-du/catkin_ws/src/ros_drone_swarm_mocap/";
static std::string test_dir = project_dir + "test/experiments/";
static std::string scripts_dir = project_dir + "scripts/";
static std::string fcpu = test_dir + "cpu.txt";
static std::string fram = test_dir + "ram.txt";
static std::string fpower = test_dir + "power.txt"; 
static std::string ftime = test_dir + "time.txt";
static std::string fdist = test_dir + "distance.txt";
static std::string fangles = test_dir + "angles.txt";

static std::string dimages = test_dir + "images/";
static ros::Time initTime;
static struct sysinfo system_info;
static int frameCounter;
#define FRAMES_BETWEEN_SAVES 10

#define D_INIT() {  initTime = ros::Time::now(); frameCounter = 0; \
                    std::ofstream myfile1(ftime);   myfile1.close(); \
                    std::ofstream myfile2(fcpu);    myfile2.close(); \
                    std::ofstream myfile3(fram);    myfile3.close(); \
                    std::ofstream myfile4(fdist);   myfile4.close();\
                    std::ofstream myfile5(fangles); myfile5.close(); \
                    std::ofstream myfile6(fpower);  myfile6.close();\
                    system((scripts_dir + "createTestFiles.sh").c_str()); } 

#define D_TIME(f, text) { std::ofstream myfile(ftime, std::ios_base::app); \
                    ros::Time beforeTime = ros::Time::now(); \
                    f; \
                    ros::Time afterTime = ros::Time::now(); \
                    myfile << (ros::Time::now()) << " : " << text << (afterTime - beforeTime) << std::endl; \
                    myfile.close(); }

#define D_CPU() {std::ofstream myfile(fcpu, std::ios_base::app); \
                    int cpu_usage = 0 ; \
                    myfile << "[" << (ros::Time::now()) << " : " << cpu_usage << std::endl; \
                    myfile.close();}

#define D_RAM() {std::ofstream myfile(fram, std::ios_base::app); \
                    sysinfo(&system_info); \
                    myfile << (ros::Time::now()) << " : " << (system_info.totalram - system_info.freeram) << " / " << system_info.totalram << std::endl; \
                    myfile.close();}

#define D_POWER() { std::ofstream myfile(fpower, std::ios_base::app); \
                myfile << (ros::Time::now()) << " : " << n << " -x: " << x << " y: " << y << std::endl; \
                myfile.close();}

#define D_DISTANCE(n, distance) { std::ofstream myfile(fdist, std::ios_base::app); \
                                    myfile << (ros::Time::now()) << " : " << n << " - " << distance << std::endl; \
                                    myfile.close();}

#define D_ANGLES(n, x, y) { std::ofstream myfile(fangles, std::ios_base::app); \
                                    myfile << (ros::Time::now()) << " : " << n << " -x: " << x << " y: " << y << std::endl; \
                                    myfile.close();}

#define DEBUG_DA(n, distance, anglex, angley){  D_DISTANCE(n, distance) \
                                                D_ANGLES(n, anglex, angley)}

#define SAVE_FRAME(frame){  std::string filename = dimages + "experiment" + std::to_string(ros::Time::now().toSec()) + ".png"; \
                            if(frameCounter % FRAMES_BETWEEN_SAVES == 0) cv::imwrite(filename, frame); \
                            frameCounter++;}
                                                

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