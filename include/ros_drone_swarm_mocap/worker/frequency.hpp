#ifndef FREQUENCY_HPP
#define FREQUENCY_HPP

#include <opencv2/opencv.hpp>

class frequency_analysis{
    private:
        const int MAX_NUM = 500;
        std::queue<float> area;
        std::queue<float> time;
    public:
        frequency_analysis();
        void add_data(float area, float time);
        float get_frequency();
};

void get_frequency_Mat(cv::Mat& img);

#endif //FREQUENCY_HPP