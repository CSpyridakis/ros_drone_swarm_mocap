#ifndef FREQUENCY_HPP
#define FREQUENCY_HPP

#include <opencv2/opencv.hpp>
#include <ctime> // for clock() and count time

#define QUEUE_SIZE 500

class frequency_analysis{
    private:
        int countUp = 0;
        int countDown = 0;

        bool lastUp = false;
        clock_t lastTimeUp;
        clock_t upTimeStart;
        double period = 0.0; 
        double calc_period(int radius);
    public:
        frequency_analysis();
        frequency_analysis(int gau, int iH, int xH, int nS, int xS, int nV, int xV, int dir);
        void update(cv::Mat& img, cv::Mat& imghsv, cv::Rect obj, cv::Rect &outObj);
        double get_frequency();
};


#endif //FREQUENCY_HPP