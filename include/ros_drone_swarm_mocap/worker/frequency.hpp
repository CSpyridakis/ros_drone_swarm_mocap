#ifndef FREQUENCY_HPP
#define FREQUENCY_HPP

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "worker/misc.hpp"
#include "ros_drone_swarm_mocap/hsv_values.h"

#include <ctime> // for clock() and count time

#define QUEUE_SIZE 500

int get_id(float period);

class frequency_analysis{
    private:
        int samplesNum = 0;
        double averagePeriod = 0;
        int countUp = 0;
        int countDown = 0;

        bool lastUp = false;
        double lastTimeUp;
        double upTimeStart;
        double period = 0.0; 
        double calc_period(int radius);
    public:
        frequency_analysis();
        frequency_analysis(int gau, int iH, int xH, int nS, int xS, int nV, int xV, int dir);
        void update(cv::Mat& img, cv::Mat& imghsv, cv::Rect obj, cv::Rect &outObj);
        double get_frequency();
        double get_period();
};

void freqUpdateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr& msg);

#endif //FREQUENCY_HPP