
#include "test-misc.hpp"

//#define RECORD
#define VIEW

// #define VIDEO 
#define CAMERA
// #define PHOTOS

std::string  photosfolderName = "n4";
#ifdef CAMERA 
std::string input_s = "0";
#else
std::string input_s = "../videos/h0.avi";
#endif

ros_drone_swarm_mocap::mocap_worker_data procData;
cv::Mat img, Unimg, tmpImg, outImg, tmp2Img;
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
    myfile << "time,x,y,r,dist,xangle,yangle" << std::endl;   
    
#ifdef RECORD    
    int codec = cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
    cv::VideoWriter out1("../videos/" + vidNa + "-proc-hsv-clean.avi", codec, 30, cv::Size(1280, 720), true);
    cv::VideoWriter out("../videos/" + vidNa + "-proc-hsv.avi", codec, 30, cv::Size(1280, 720), true);
#endif

#ifdef VIEW
    createTrackers();
#endif
    // setupTrackers(9,26,114,97,172,77,161,1);//(5,0,36,120,168,129,255,1);     //5,26,255,123,255,103,255,1 // 5,26,56,123,255,103,255,1
    setupTrackers(11,26,38,129,212,77,202,1);//(5,0,36,120,168,129,255,1);     //5,26,255,123,255,103,255,1 // 5,26,56,123,255,103,255,1

    // Open Video
    repeatPoint:
    cv::VideoCapture cap;
    if(videoName == "0"){
        cap.open(0, cv::CAP_V4L2);
        setCameraCaptureProperties(cap);
    } 
    else {
        cap.open(videoName);
        setCameraCaptureProperties(cap);
    }
    if (!cap.isOpened()){ printf("Cannot open camera\n"); return -1;} 

    size_t img_count = 0;
    int curImg = 0;

#ifdef PHOTOS    
    std::vector<cv::String> fn;
    cv::glob("/home/cs-du/catkin_ws/src/ros_drone_swarm_mocap/test/hsv/img/" + photosfolderName +"/*.jpg", fn, false);

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
        undDistord(img, Unimg);
        cv::undistort(img, Unimg, camCalib, distCoef);
        // cv::imshow("Undistorted", Unimg);
        tmpImg = Unimg.clone();
        
        findBallAndDisplay(procData, tmpImg);

#ifdef VIEW
        cv::imshow("Out Image", tmpImg); 
        char key = cv::waitKey(30);
        if ( key == 27){ printf("Esc key is pressed by user. Exit!\n"); goto exitPoint;}
        if ( key == 'p' ) {playvideo = !playvideo;}
        if ( key == 'n' ) { if(curImg + 1 < img_count) curImg ++; else break; }
        if ( key == 's' ) { 
#ifdef PHOTOS
            std::string outName = fn[curImg] + "-proc.jpg";
            // std::cout << outName << std::endl;
            cv::imwrite(outName, tmpImg);
            myfile << "-" << "," << procData.balls[0].image_plane_x << "," << procData.balls[0].image_plane_y << "," << procData.balls[0].image_plane_r << "," << procData.balls[0].distance_from_camera << "," << procData.balls[0].xangle << "," << procData.balls[0].yangle << std::endl;
#else
            curImg++; 
            cv::imwrite("/home/cs-du/catkin_ws/src/ros_drone_swarm_mocap/test/hsv/img/out/" + std::to_string(curImg) + ".jpg", tmpImg);
            myfile << std::to_string((clock() - beforeTime) / (double) CLOCKS_PER_SEC) << "," << procData.balls[0].distance_from_camera << "," << procData.balls[0].xangle << "," << procData.balls[0].yangle << std::endl;
#endif
        }

#ifdef RECORD
myfile << std::to_string((clock() - beforeTime) / (double) CLOCKS_PER_SEC) << "," << procData.balls[0].distance_from_camera << "," << procData.balls[0].xangle << "," << procData.balls[0].yangle << std::endl;
out << tmpImg;
out1 << Unimg;
// std::cout << "." << std::flush;
#endif

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