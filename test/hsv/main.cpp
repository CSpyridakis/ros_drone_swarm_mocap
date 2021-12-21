
#include <iostream>
#include <opencv2/opencv.hpp>
#include "worker/hsvDetection.hpp"
#include "worker/drawInfoToImage.hpp"

// For undistortion 1280,720
static float cameraCalibrationdata[9] = {9.113812935295416e+02, 0.651033616843436, 6.644831723997970e+02, 0, 9.113086377881884e+02, 3.713670194501918e+02,  0, 0, 1};
static float distCoeffsCalibrationdata[5] = {-0.032436584225176, 0.087146504956371, -4.167775601669913e-04, -4.631801852683015e-04, -0.073076772853399};
cv::Mat camCalib = cv::Mat(3, 3, CV_32F, cameraCalibrationdata);
cv::Mat distCoef = cv::Mat(1, 5, CV_32F, distCoeffsCalibrationdata);

// =========================================================================
float objRealSize_m =  0.144;
float sensorsize_mm = 755.875793;

int main(int argc, char** argv ){
    std::string videoName = "0";
    int videoNum = -1;
    if (argc == 2) {
        videoNum = atoi(argv[1]);
    } 

    float xfocalLengthin_mm = cameraCalibrationdata[0];
    float yfocalLengthin_mm = cameraCalibrationdata[4];
    int xFOVAngles;
    int yFOVAngles;
    
    // int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    // cv::VideoWriter out(videoName + "-proc-hsv.avi", codec, 30, cv::Size(1280, 720), true);

    // Open Video
    cv::VideoCapture cap;
    if(videoName == "0"){
        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));      // More fps less resolution (at least for my setup)
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap.set(cv::CAP_PROP_FPS, 30);
        int dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); 
        int dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        int fps_counter = cap.get(cv::CAP_PROP_FPS);
        printf("After set, actual resolution of the video w:%d h:%d fps:%d\n", dWidth, dHeight, fps_counter); 

        xFOVAngles = 2 * atan((float)dWidth  / (2 * (float)xfocalLengthin_mm)) * 180.0 / 3.14159265358979323846;
        yFOVAngles = 2 * atan((float)dHeight / (2 * (float)yfocalLengthin_mm)) * 180.0 / 3.14159265358979323846;
        printf("xFOVangles: %d  yFOVangles: %d\n", xFOVAngles, yFOVAngles);
    }
    else
        cap.open(videoName);
    if (!cap.isOpened()){ printf("Cannot open camera\n"); return -1;} 
    
    cv::Mat img, Unimg, tmpImg, outImg;
    bool playvideo = true;
    while(true){
        if (playvideo && !cap.read(img)){printf("Input has disconnected\n"); break;}
        cv::undistort(img, Unimg, camCalib, distCoef);
        // cv::imshow("Undistorted", Unimg);
        tmpImg = Unimg.clone();

        std::vector<cv::Vec3f> circles;
        hsvDetection(tmpImg, circles);
        
        cameraPrintInfo(tmpImg, 1);
        // drawCircles(imgProcDebug, imgProcDebug, procData);

        cv::imshow("Out Image", tmpImg); 
        char key = cv::waitKey(1); 
        if ( key == 27){ printf("Esc key is pressed by user. Exit!\n"); break;}
        if ( key == 'p' ) {playvideo = !playvideo;}
    }
    // out.release();
    
    return 0;
}