
#include "test-misc.hpp"

//#define RECORD
#define VIEW

// #define VIDEO 
#define CAMERA
// #define PHOTOS
#ifdef CAMERA 
std::string input_s = "0";
#else
std::string input_s = "../videos/t1.avi";
#endif

ros_drone_swarm_mocap::mocap_worker_data procData;
cv::Mat img, Unimg, tmpImg, outImg;
bool playvideo = true;

int main(int argc, char** argv ){
    std::cout << "Welcome" << std::endl << std::flush;

    std::string videoName = input_s;
    int videoNum = -1;
    srand(time(NULL));
    
    procDataParams(procData);

    std::string vidNa = std::to_string(rand());
    std::string filename = "../" + vidNa + ".csv";
    std::ofstream myfile(filename, std::ios_base::app);
    myfile << "time,dist,xangle,yangle" << std::endl;   
    
#ifdef RECORD    
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoWriter out1("../videos/" + vidNa + "-proc-hsv-clean.avi", codec, 30, cv::Size(1280, 720), true);
    cv::VideoWriter out("../videos/" + vidNa + "-proc-hsv.avi", codec, 30, cv::Size(1280, 720), true);
#endif

#ifdef VIEW
    createTrackers();
#endif
    setupTrackers(5,26,56,123,255,103,255,1);     // 5,26,56,123,255,103,255,1

    // Open Video
    repeatPoint:
    cv::VideoCapture cap;
    if(videoName == "0")
        setCameraCaptureProperties(cap);
    else
        cap.open(videoName);
    if (!cap.isOpened()){ printf("Cannot open camera\n"); return -1;} 

    size_t img_count = 0;
    int curImg = 0;

#ifdef PHOTOS    
    std::vector<cv::String> fn;
    cv::glob("/home/cs-du/catkin_ws/src/ros_drone_swarm_mocap/test/hsv/img/node1/*.jpg", fn, false);

    std::vector<cv::Mat> images;
    img_count = fn.size(); 
#endif
    
    const std::clock_t beforeTime = clock();
    while(true){
#ifdef VIEW
        fixTrackers();
#endif
#ifdef PHOTOS
        img = cv::imread(fn[curImg]);
#else
        if (playvideo && !cap.read(img)){printf("Input has disconnected or video ended\n"); break;}
#endif
        cv::undistort(img, Unimg, camCalib, distCoef);
        // cv::imshow("Undistorted", Unimg);
        tmpImg = Unimg.clone();

        findBallAndDisplay(procData, tmpImg);

#ifdef PHOTOS
        std::string outName = fn[curImg] + "-proc.jpg";
        std::cout << outName << std::endl;
        cv::imwrite(outName, tmpImg);
        myfile << std::to_string((clock() - beforeTime) / (double) CLOCKS_PER_SEC) << "," << procData.balls[0].distance_from_camera << "," << procData.balls[0].xangle << "," << procData.balls[0].yangle << std::endl;
#endif

#ifdef RECORD
        myfile << std::to_string((clock() - beforeTime) / (double) CLOCKS_PER_SEC) << "," << procData.balls[0].distance_from_camera << "," << procData.balls[0].xangle << "," << procData.balls[0].yangle << std::endl;
        out << tmpImg;
        out1 << Unimg;
        std::cout << "." << std::flush;
#endif

#ifdef VIEW
        cv::imshow("Out Image", tmpImg); 
#ifdef PHOTOS
        char key = cv::waitKey(0); 
#else
        char key = cv::waitKey(30);
#endif
        if ( key == 27){ printf("Esc key is pressed by user. Exit!\n"); goto exitPoint;}
        if ( key == 'p' ) {playvideo = !playvideo;}
        if ( key == 'n' ) { if(curImg + 1 < img_count) curImg ++; else break; }
        if ( key == 's' ) { 
            curImg++; 
            cv::imwrite("../img/" + std::to_string(curImg) + ".jpg", tmpImg);
            myfile << std::to_string((clock() - beforeTime) / (double) CLOCKS_PER_SEC) << "," << procData.balls[0].distance_from_camera << "," << procData.balls[0].xangle << "," << procData.balls[0].yangle << std::endl;
        }
#endif
    
    }
#ifndef RECORD
    goto repeatPoint;
#endif
    exitPoint:
#ifdef RECORD
    out.release();
    out1.release();
#endif

    myfile.close();

    return 0;
}