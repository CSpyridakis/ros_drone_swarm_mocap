#include "worker/frequency.hpp"
#define X_OFFSET 20
#define Y_OFFSET 20
#define MAX_NUM 300

frequency_analysis::frequency_analysis(){
    for(int i=0;i<MAX_NUM;i++){
        area.push(0.0);
        time.push(0.0);
    }
}

void frequency_analysis::add_data(float area_d, float time_d){
    if(area.size() + 1 > MAX_NUM){
        area.pop();
    }
    area.push(area_d);

    if(time.size() + 1 > MAX_NUM){
        time.pop();
    }
    time.push(time_d);
}

float frequency_analysis::get_frequency(){

}

void get_frequency_Mat(cv::Mat& img){
    // Mat = 

    for (int i = 0 ; i< MAX_NUM; i++){
        cv::circle(img, cv::Point(i+X_OFFSET, Y_OFFSET), 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
}