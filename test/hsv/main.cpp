/**
 * IMPORTANT! You have to have all ROS_INFO commented out to use this code!
*/

#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <cstring>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "worker/extendDIstAng.hpp"
#include "ros_drone_swarm_mocap/mocap_worker_data.h"
#include "worker/hsvDetection.hpp"
#include "worker/drawInfoToImage.hpp"

// For undistortion 1280,720
static float cameraCalibrationdata[9] = {9.113812935295416e+02, 0.651033616843436, 6.644831723997970e+02, 0, 9.113086377881884e+02, 3.713670194501918e+02,  0, 0, 1};
static float distCoeffsCalibrationdata[5] = {-0.032436584225176, 0.087146504956371, -4.167775601669913e-04, -4.631801852683015e-04, -0.073076772853399};
cv::Mat camCalib = cv::Mat(3, 3, CV_32F, cameraCalibrationdata);
cv::Mat distCoef = cv::Mat(1, 5, CV_32F, distCoeffsCalibrationdata);

// =========================================================================
float objRealSize_m =  0.144;
float xsensorsize_mm = 755.875793;
float ysensorsize_mm = 755.875793;

#define IMAGE_W 1280
#define IMAGE_H 720

int main(int argc, char** argv ){
    std::string videoName = "0";
    int videoNum = -1;
    if (argc == 2) {
        videoNum = atoi(argv[1]);
    } 
    srand(time(NULL));

    ros_drone_swarm_mocap::mocap_worker_data procData;
    procData.nodeID = 1;
    procData.camera.imageHeightInPixels = IMAGE_H;
    procData.camera.imageWidthInPixels =IMAGE_W;
    procData.camera.XfocalLengthInMillimeters = cameraCalibrationdata[0];
    procData.camera.YfocalLengthInMillimeters = cameraCalibrationdata[4];
    procData.camera.XFieldOfViewInAngles = 2 * atan((float)procData.camera.imageWidthInPixels  / (2 * (float)procData.camera.XfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.YFieldOfViewInAngles = 2 * atan((float)procData.camera.imageHeightInPixels / (2 * (float)procData.camera.YfocalLengthInMillimeters)) * 180.0 / CV_PI ;
    procData.camera.objectsRealSizeInMeter = objRealSize_m;
    procData.camera.XsensorSizeInMillimeters = xsensorsize_mm;
    procData.camera.YsensorSizeInMillimeters = ysensorsize_mm;
    // procData.pose.x = node_x;
    // procData.pose.y = node_y;
    // procData.pose.z = node_z;
    // procData.pose.roll = node_roll;
    // procData.pose.pitch = node_pitch;
    // procData.pose.yaw = node_yaw;
    
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    std::string vidNa = std::to_string(rand());
    cv::VideoWriter out1("../videos/" + vidNa + "-proc-hsv-clean.avi", codec, 30, cv::Size(1280, 720), true);
    cv::VideoWriter out("../videos/" + vidNa + "-proc-hsv.avi", codec, 30, cv::Size(1280, 720), true);

    // Open Video
    cv::VideoCapture cap;
    if(videoName == "0"){
        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));      // More fps less resolution (at least for my setup)
        cap.set(cv::CAP_PROP_FRAME_WIDTH, IMAGE_W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMAGE_H);
        cap.set(cv::CAP_PROP_FPS, 30);
        int dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); 
        int dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        int fps_counter = cap.get(cv::CAP_PROP_FPS);
        printf("After set, actual resolution of the video w:%d h:%d fps:%d\n", dWidth, dHeight, fps_counter); 
    }
    else
        cap.open(videoName);
    if (!cap.isOpened()){ printf("Cannot open camera\n"); return -1;} 
    
    cv::Mat img, Unimg, tmpImg, outImg;
    bool playvideo = true;
    std::string filename = "../videos/" + vidNa + ".csv";
    std::ofstream myfile(filename, std::ios_base::app);
    myfile << "time,dist,xangle,yangle" << std::endl; 
    const std::clock_t beforeTime = clock();
    while(true){
        procData.balls.clear();
        if (playvideo && !cap.read(img)){printf("Input has disconnected\n"); break;}
        cv::undistort(img, Unimg, camCalib, distCoef);
        // cv::imshow("Undistorted", Unimg);
        tmpImg = Unimg.clone();

        std::vector<cv::Vec3f> circles;
        hsvDetection(tmpImg, circles);
        saveDistancesToProcData(circles, procData);
        cameraPrintInfo(tmpImg, 1);
        drawCircles(tmpImg, tmpImg, procData);

        out << tmpImg;
        out1 << Unimg;
        myfile << std::to_string((clock() - beforeTime) / (double) CLOCKS_PER_SEC) << "," << procData.balls[0].distance_from_camera << "," << procData.balls[0].xangle << "," << procData.balls[0].yangle << std::endl;

        // cv::imshow("Out Image", tmpImg); 
        char key = cv::waitKey(1); 
        if ( key == 27){ printf("Esc key is pressed by user. Exit!\n"); break;}
        if ( key == 'p' ) {playvideo = !playvideo;}
    }
    out.release();
    out1.release();
    myfile.close();
    
    return 0;
}