#include "worker/frequency.hpp"
#include <sys/time.h>

#define ALL_NODES 0

#define X_OFFSET 20
#define Y_OFFSET 20

static int gaussian_kernel_size = 5;
static int minH = 40;
static int minS = 70;
static int minV = 150;
static int maxH = 60;
static int maxS = 120;
static int maxV = 255;
static int di_er_kernel = 1;

void freqUpdateHSVvaluesCallback(const ros_drone_swarm_mocap::hsv_values::ConstPtr& msg){
    if (msg->id == 1 || msg->id == ALL_NODES ){  //TODO: change 
        // if input gaussian kernel size is positive and 
        gaussian_kernel_size = (msg->gaussian_kernel_size > 0 && msg->gaussian_kernel_size% 2 != 0) ? 
                                msg->gaussian_kernel_size : gaussian_kernel_size + 1 ;

        minH                = msg->minH;
        maxH                = msg->maxH;
        minS                = msg->minS;
        maxS                = msg->maxS;  
        minV                = msg->minV;
        maxV                = msg->maxV;
        di_er_kernel        = msg->di_er_kernel;
        if (di_er_kernel % 2 == 0 ) di_er_kernel = di_er_kernel > 0 ? di_er_kernel + 1 : 1;
        ROS_INFO("Frequency: Got new HSV values Min(%d, %d, %d), Max(%d, %d, %d) | di_er_kernel: %d, gaussian_kernel_size: %d", minH, minS, minV, maxH, maxS, maxV, di_er_kernel, gaussian_kernel_size);
    }
}

static double gettime() {
    struct timeval ttime;
    gettimeofday(&ttime, NULL);
    return ttime.tv_sec + ttime.tv_usec * 0.000001;
}

#ifdef TESTING_HSV
static void createTrackersFreq(){
    cv::namedWindow("frequency-trackbars", cv::WINDOW_AUTOSIZE); // Create Window
    cv::createTrackbar( "gaussian", "frequency-trackbars", &gaussian_kernel_size, 20);
    cv::createTrackbar( "minH", "frequency-trackbars", &minH, 255);
    cv::createTrackbar( "maxH", "frequency-trackbars", &maxH, 255);
    cv::createTrackbar( "minS", "frequency-trackbars", &minS, 255);
    cv::createTrackbar( "maxS", "frequency-trackbars", &maxS, 255);
    cv::createTrackbar( "minV", "frequency-trackbars", &minV, 255);
    cv::createTrackbar( "maxV", "frequency-trackbars", &maxV, 255); 
    cv::createTrackbar( "di_er_kernel", "frequency-trackbars", &di_er_kernel, 20); 
}
static void fixTrackersFreq(){
        if(di_er_kernel % 2 == 0)
            cv::setTrackbarPos("di_er_kernel", "frequency-trackbars", di_er_kernel+1);

        if(gaussian_kernel_size % 2 == 0)
            cv::setTrackbarPos("gaussian", "frequency-trackbars", gaussian_kernel_size+1);
}
static void setupTrackersFreq(int gau, int iH, int xH, int nS, int xS, int nV, int xV, int dir){
    cv::setTrackbarPos( "gaussian", "frequency-trackbars", gau);
    cv::setTrackbarPos( "minH", "frequency-trackbars", iH);
    cv::setTrackbarPos( "maxH", "frequency-trackbars", xH);
    cv::setTrackbarPos( "minS", "frequency-trackbars", nS);
    cv::setTrackbarPos( "maxS", "frequency-trackbars", xS);
    cv::setTrackbarPos( "minV", "frequency-trackbars", nV);
    cv::setTrackbarPos( "maxV", "frequency-trackbars", xV); 
    cv::setTrackbarPos( "di_er_kernel", "frequency-trackbars", dir); 
}

#endif

frequency_analysis::frequency_analysis(){
}

frequency_analysis::frequency_analysis(int gau, int iH, int xH, int nS, int xS, int nV, int xV, int dir){
    frequency_analysis();
#ifdef TESTING_HSV
    createTrackersFreq();
    setupTrackersFreq(gau, iH, xH, nS, xS, nV, xV, dir);
#endif
}

double frequency_analysis::calc_period(int radius){
    // static bool lowFirst = true;

    if(lastUp && radius == 0){
        upTimeStart = gettime();
        lastUp = false;
        // std::cout << "New: ";
    }
    else if(lastUp && radius > 0){
        // std::cout << "^";
    }
    else if(!lastUp && radius > 0){
        lastUp = true;
        period = (double) (gettime() - upTimeStart);
        // std::cout << "  " << period << std::endl;
    }
    else if(!lastUp && radius == 0){
        // std::cout << ".";
    }
}

int get_id(float period){
    //std::cout << period << std::endl;
    for (int i = 0; i<10; i++)
        if (period > (double) (i*1.0) && period <= (double)((i+1)*1.0))
            return i+1;
    return 0;
}

double frequency_analysis::get_period(){
    return period;
}

double frequency_analysis::get_frequency(){
    return 1/period;
}

void frequency_analysis::update(cv::Mat& img, cv::Mat& imghsv, cv::Rect obj, cv::Rect &outObj){
#ifdef TESTING_HSV
    fixTrackersFreq();
#endif
    cv::Mat mask, tmpImg;
    imghsv(obj).copyTo(tmpImg);

    if(tmpImg.empty()) return;

    cv::inRange(tmpImg, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);
    cv::Mat kernelDiER = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(di_er_kernel, di_er_kernel));
    cv::dilate(mask, mask, kernelDiER);
    cv::erode(mask, mask, kernelDiER);

    cv::Mat Points;
    cv::findNonZero(mask,Points);
    cv::Rect ballBound = boundingRect(Points);
    cv::Point ballCenter = (ballBound.br() + ballBound.tl())*0.5;
    int radius = ballBound.height > ballBound.width ? ballBound.height/2 : ballBound.width/2;

    // cv::imshow("Rdl", mask);

    outObj = cv::Rect(obj.tl() + ballBound.tl(), obj.tl() + ballBound.br());

    calc_period(radius);
}