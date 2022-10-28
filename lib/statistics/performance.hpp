/**
 * @brief   Use this header file ONLY to test MPU performance, then REMOVE IT!
 *          This file has not been optimized!!!
 */
#ifndef PERFORMANCE_HPP
#define PERFORMANCE_HPP

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <iostream>
#include <ctime>
#include <cstring>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <sys/time.h>

static double gettime() {
    struct timeval ttime;
    gettimeofday(&ttime, NULL);
    return ttime.tv_sec + ttime.tv_usec * 0.000001;
}

#ifdef __GNUC__
#define VARIABLE_IS_NOT_USED __attribute__ ((unused))
#else
#define VARIABLE_IS_NOT_USED
#endif
static int rnadnum = 0;

// TODO: Maybe comment out 
#define RASPBERRY

// TODO: you may need to change it
#ifndef RASPBERRY
#define PROJECT_DIR "/home/cs-du/catkin_ws/src/ros_drone_swarm_mocap/" 
#else
#define PROJECT_DIR "/home/ubuntu/catkin_ws/src/ros_drone_swarm_mocap/" 
#endif
#define TEST_DIR PROJECT_DIR "test/experiments/"
#define SCRIPTS_DIR PROJECT_DIR "scripts/"

//FIXME: you may need to change this interface
//IMPORTANT: This produces error if not set properly!
#ifndef RASPBERRY
#define NET_INTERFACE "enp24s0"
#else
#define NET_INTERFACE "wlan0"
#endif

typedef struct file_info{
    std::string name;
    std::string fileheader;
}file_info;

#define _durations 0
#define _cpu 1
#define _ram 2
#define _power 3 
#define _temp 4
#define _mdist 5 
#define _angle 6
#define _net 7

// TODO: you may need to change these files names
const static file_info f[] = {
    {"durations.csv", "time,function,duration"},
    {"cpu.csv", "time,usage"},
    {"ram.csv", "time,used,free"},
    {"power.csv", "time,power"},
    {"temp.csv", "time,temp"},
    {"distance.csv", "time,node,distance"},
    {"angles.csv", "time,node,x,y"},
    {"network.csv", "time,up,down"},
};
#define file_info_size (int)(sizeof(f)/sizeof(file_info)) 

std::string to_string_with_precision(float value, const int n);

// =============================================================================================
//                                   SAVE FRAMES FROM VIDEO
// =============================================================================================
// FIXME: This is not used to the generic performance lib
VARIABLE_IS_NOT_USED static int frameCounter;
#define FRAMES_BETWEEN_SAVES 30     // TODO: Maybe you need to change this MACRO
static std::string dimages = std::string(TEST_DIR) + "images/";                     /* The file to be saved */

#define SAVE_FRAME(eventMoment, frame){ \
    std::string filename = dimages + std::to_string(eventMoment) + ".png"; \
    if(frameCounter % FRAMES_BETWEEN_SAVES == 0) cv::imwrite(filename, frame); \
    frameCounter++; \
}

// =============================================================================================
//                                   CALCULATE TIME OF FUNCTION EXECUTION
// =============================================================================================
  
const static std::string ftime = std::string(TEST_DIR) + f[_durations].name;         /* The file to be saved */
#define D_TIME(eventMoment, function, text) { \
    std::ofstream myfile(ftime, std::ios_base::app); \
    double beforeTime = gettime(); \
    function; \
    double duration = (gettime() - beforeTime); \
    myfile << std::to_string(eventMoment) << "," << text << "," << duration << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                            CPU USAGE
// =============================================================================================

static std::string fcpu = std::string(TEST_DIR) + f[_cpu].name;                       /* The file to be saved */

typedef struct cpu_stat{
    long double user;
    long double nice;
    long double system;
    long double idle;
    long double iowait;
    long double irq;
    long double softirq;
    long double steal;
}cpu_stat;
void read_cpu_stats(cpu_stat &stats);
double cpu_usage_get(cpu_stat &prev_stats, cpu_stat &now_stats);

typedef struct cpu_usage{
    long double used;
}cpu_usage;
void get_cpu_usage(cpu_usage &cu);

VARIABLE_IS_NOT_USED static cpu_stat preStatsD = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
VARIABLE_IS_NOT_USED static cpu_stat nowStatsD;

#define D_CPU(eventMoment) { \
    std::ofstream myfile(fcpu, std::ios_base::app); \
    read_cpu_stats(nowStatsD); \
    float cpuUsage = cpu_usage_get(preStatsD, nowStatsD); \
    myfile << std::to_string(eventMoment) << "," << cpuUsage << std::endl; \
    memcpy(&preStatsD, &nowStatsD, sizeof(cpu_stat)); \
    myfile.close(); \
}

// =============================================================================================
//                                             RAM USAGE
// =============================================================================================

static std::string fram = std::string(TEST_DIR) + f[_ram].name;                       /* The file to be saved */
typedef struct ram_usage{
    long double used;
    long double free;
    long double total;
}ram_usage;

ram_usage get_ram_usage();

#define D_RAM(eventMoment) { \
    std::ofstream myfile(fram, std::ios_base::app); \
    ram_usage ru = get_ram_usage(); \
    myfile << std::to_string(eventMoment) << "," << ru.used << "," << ru.total << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                             NETWORK
// =============================================================================================
static std::string fnet = std::string(TEST_DIR) + f[_net].name;

typedef struct net_usage{
    long double down;
    long double up;
    std::clock_t last_read;
}net_usage;
static net_usage prevNetD;

long double networkRead(std::string interface, std::string rt_x);

void get_net_stat(net_usage &stats);

net_usage get_net_usage();

#define D_NET(eventMoment){ \
    double timeFromLast = (clock() - prevNetD.last_read) / (double) CLOCKS_PER_SEC; \
    std::ofstream myfile(fnet, std::ios_base::app); \
    if(timeFromLast >= 1.0){ \
        net_usage nowNet; \
        get_net_stat(nowNet); \
        myfile << std::to_string(eventMoment) << "," << std::to_string(nowNet.up) << "," << std::to_string(nowNet.down) << std::endl; \
    } else myfile << std::to_string(eventMoment) << "," << std::to_string(0.0) << "," << std::to_string(0.0) << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                             POWER
// =============================================================================================

// TODO: 
static std::string fpower = std::string(TEST_DIR) + f[_power].name;                 /* The file to be saved */
#define D_POWER(eventMoment) { \
    std::ofstream myfile(fpower, std::ios_base::app); \
    myfile << std::to_string(eventMoment) << "," << n << "," << x << "," << y << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                          TEMPERATURE
// =============================================================================================

static std::string ftemp = std::string(TEST_DIR) + f[_temp].name;                   /* The file to be saved */

// #define GET_TEMP    "$(sensors | grep 'C' | grep 'temp\\|Tctl\\|Package' | " \
//                     "head -n 2 | grep -o -E '[0-9][0-9].[0-9].C'| tr '\\n' ' ' | " \
//                     "tr -s ' ' | cut -d' ' -f1 | head -c -3 | grep -o -E '[0-9][0-9].[0-9]')" 

#define GET_TEMP "$(sensors | grep 'C' | grep 'temp\\|Tctl\\|Package' | tr '\n' ' ' | tr -s ' ' | cut -d' ' -f2 | head -c 5 | tail -c 4)"

#define D_TEMP(eventMoment) { \
    std::string command = "echo \"" + std::to_string(eventMoment) + "," + GET_TEMP + "\" >> " + ftemp; \
    system(command.c_str()); \
}

// =============================================================================================
//                                      DISTANCE + ANGLE
// =============================================================================================

static std::string fdist = std::string(TEST_DIR) + f[_mdist].name;  /* The file to be saved */
#define D_DISTANCE(eventMoment, n, distance) { \
    std::ofstream myfile(fdist, std::ios_base::app); \
    myfile << std::to_string(eventMoment) << "," << n << "," << distance << std::endl; \
    myfile.close(); \
}

static std::string fangles = std::string(TEST_DIR) + f[_angle].name;
#define D_ANGLES(eventMoment, n, x, y) { \
    std::ofstream myfile(fangles, std::ios_base::app); \
    myfile << std::to_string(eventMoment) << "," << n << "," << x << "," << y << std::endl; \
    myfile.close(); \
}

#define DEBUG_DA(eventMoment, n, distance, anglex, angley){ \
    D_DISTANCE(eventMoment, n, distance); \
    D_ANGLES(eventMoment, n, anglex, angley); \
}

// =============================================================================================
//                                          INIT
// =============================================================================================

VARIABLE_IS_NOT_USED static std::string get_filenames(){
    std::string retStr = "";
    for(int i=0; i<file_info_size; i++)
        retStr += f[i].name + " ";
    return retStr;
}

#define D_INIT() {  \
    srand(time(NULL)); \
    rnadnum = rand(); \
    frameCounter = 0; \
    std::string command = std::string(SCRIPTS_DIR) + "createTestFiles.sh " + get_filenames(); \
    system(command.c_str()); \
    read_cpu_stats(preStatsD); \
    std::ofstream myfile; \
    for(int i=0; i<file_info_size; i++){ \
        myfile.open(f[i].name, std::ios_base::app); \
        myfile << f[i].fileheader.c_str() << std::endl; \
        myfile.close(); \
    } \
}        

#endif //PERFORMANCE_HPP  
