#ifndef PERFORMANCE_HPP
#define PERFORMANCE_HPP

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <iostream>
#include <ctime>

#ifdef __GNUC__
#define VARIABLE_IS_NOT_USED __attribute__ ((unused))
#else
#define VARIABLE_IS_NOT_USED
#endif

static std::string project_dir = "/home/ubuntu/catkin_ws/src/ros_drone_swarm_mocap/"; // TODO: change it
static std::string test_dir = project_dir + "test/experiments/";
static std::string scripts_dir = project_dir + "scripts/";

typedef struct file_info{
    std::string name;
    std::string fileheader;
}file_info;

#define _durations 0
#define _cpu 1
#define _ram 2
#define _power 3 
#define _temp 4
#define _distance 5 
#define _angle 6
#define _net 7

// TODO: you may need to change these filesnames
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

// =============================================================================================
//                                   SAVE FRAMES FROM VIDEO
// =============================================================================================
// FIXME: This is not used to the generic performance lib
VARIABLE_IS_NOT_USED static int frameCounter;
#define FRAMES_BETWEEN_SAVES 30     // TODO: Maybe you need to change this MACRO
static std::string dimages = test_dir + "images/";                     /* The file to be saved */

#define SAVE_FRAME(eventMoment, frame){ \
    std::string filename = dimages + std::to_string(eventMoment) + ".png"; \
    if(frameCounter % FRAMES_BETWEEN_SAVES == 0) cv::imwrite(filename, frame); \
    frameCounter++; \
}

// =============================================================================================
//                                   CALCULATE TIME OF FUNCTION EXECUTION
// =============================================================================================
  
const static std::string ftime = test_dir + f[_durations].name;         /* The file to be saved */
#define D_TIME(eventMoment, function, text) { \
    std::ofstream myfile(ftime, std::ios_base::app); \
    const std::clock_t beforeTime = clock(); \
    function; \
    double duration = (clock() - beforeTime) / (double) CLOCKS_PER_SEC; \
    myfile << std::to_string(eventMoment) << "," << text << "," << duration << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                            CPU USAGE
// =============================================================================================

static std::string fcpu = test_dir + f[_cpu].name;                       /* The file to be saved */

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
double cpu_usage(cpu_stat &prev_stats, cpu_stat &now_stats);
    
VARIABLE_IS_NOT_USED static cpu_stat preStats = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
VARIABLE_IS_NOT_USED static cpu_stat nowStats;

#define D_CPU(eventMoment) { \
    std::ofstream myfile(fcpu, std::ios_base::app); \
    read_cpu_stats(nowStats); \
    float cpuUsage = cpu_usage(preStats, nowStats); \
    myfile << std::to_string(eventMoment) << "," << cpuUsage << std::endl; \
    memcpy(&preStats, &nowStats, sizeof(cpu_stat)); \
    myfile.close(); \
}

// =============================================================================================
//                                             RAM USAGE
// =============================================================================================

static std::string fram = test_dir + f[_ram].name;                       /* The file to be saved */
#define D_RAM(eventMoment) { \
    static struct sysinfo system_info; \
    std::ofstream myfile(fram, std::ios_base::app); \
    sysinfo(&system_info); \
    myfile << std::to_string(eventMoment) << "," << (system_info.totalram - system_info.freeram) << "," << system_info.totalram << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                             NETWORK
// =============================================================================================
static std::string fnet = test_dir + f[_net].name;

typedef struct net_stat{
    long double down;
    long double up;
    std::clock_t last_read;
}net_stat;
static net_stat prevNet;
long double networkRead(std::string interface, std::string rt_x);

VARIABLE_IS_NOT_USED static void get_net_stat(net_stat &stats, std::string interface){
    double netnowUp = networkRead(interface.c_str(), "tx"); 
    double netnowDown = networkRead(interface.c_str(), "rx");  
    stats.up = netnowUp - prevNet.up;
    stats.down = netnowDown -  prevNet.down;
    prevNet.up = netnowUp;
    prevNet.down = netnowDown;
    prevNet.last_read = clock();
}

#define D_NET(eventMoment, interface){ \
    double timeFromLast = (clock() - prevNet.last_read) / (double) CLOCKS_PER_SEC; \
    std::ofstream myfile(fnet, std::ios_base::app); \
    if(timeFromLast >= 1.0){ \
        net_stat nowNet; \
        get_net_stat(nowNet, interface); \
        myfile << std::to_string(eventMoment) << "," << std::to_string(nowNet.up) << "," << std::to_string(nowNet.down) << std::endl; \
    } else myfile << std::to_string(eventMoment) << "," << std::to_string(0.0) << "," << std::to_string(0.0) << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                             POWER
// =============================================================================================

// TODO: 
static std::string fpower = test_dir + f[_power].name;                 /* The file to be saved */
#define D_POWER(eventMoment) { \
    std::ofstream myfile(fpower, std::ios_base::app); \
    myfile << std::to_string(eventMoment) << "," << n << "," << x << "," << y << std::endl; \
    myfile.close(); \
}

// =============================================================================================
//                                          TEMPERATURE
// =============================================================================================

static std::string ftemp = test_dir + f[_temp].name;                   /* The file to be saved */

#define GET_TEMP    "$(sensors | grep 'C' | grep 'temp\\|Tctl\\|Package' | " \
                    "head -n 2 | grep -o -E '[0-9][0-9].[0-9].C'| tr '\\n' ' ' | " \
                    "tr -s ' ' | cut -d' ' -f1 | head -c -3 | grep -o -E '[0-9][0-9].[0-9]')" 

#define D_TEMP(eventMoment) { \
    std::string command = "echo \"" + std::to_string(eventMoment) + "," + GET_TEMP + "\" >> " + ftemp; \
    system(command.c_str()); \
}

// =============================================================================================
//                                      DISTANCE + ANGLE
// =============================================================================================

static std::string fdist = test_dir + f[_distance].name;              /* The file to be saved */
#define D_DISTANCE(eventMoment, n, distance) { \
    std::ofstream myfile(fdist, std::ios_base::app); \
    myfile << std::to_string(eventMoment) << "," << n << "," << distance << std::endl; \
    myfile.close(); \
}

static std::string fangles = test_dir + f[_angle].name;
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
    frameCounter = 0; \
    std::string command = scripts_dir + "createTestFiles.sh " + get_filenames(); \
    system(command.c_str()); \
    read_cpu_stats(preStats); \
    std::ofstream myfile; \
    for(int i=0; i<file_info_size; i++){ \
        myfile.open(f[i].name, std::ios_base::app); \
        myfile << f[i].fileheader.c_str() << std::endl; \
        myfile.close(); \
    } \
}        

#endif //PERFORMANCE_HPP  
