#include <opencv2/core/fast_math.hpp>

#include "worker/extendDistAng.hpp"
#include "distance-angle/distance-angle.hpp"
#include "statistics/performance.hpp"
#include "worker/misc.hpp"
#include "worker/frequency.hpp"

float calculateSensorSize(  int objectSizeInPixels, 
                            float objectsDistanceFromCameraInMeters, 
                            ros_drone_swarm_mocap::mocap_worker_data& procData){
    
    float retVal = calculateSensorsSizeFull(procData.camera.XfocalLengthInMillimeters, 
                                    procData.camera.objectsRealSizeInMeter, 
                                    procData.camera.imageWidthInPixels,
                                    objectSizeInPixels, 
                                    objectsDistanceFromCameraInMeters);
#ifdef TESTING_HSV
    // std::cout << "Focal length (mm): " << procData.camera.XfocalLengthInMillimeters <<  
    //             " | Obj size (m): " << procData.camera.objectsRealSizeInMeter << 
    //             " | Img width (pixels): " << procData.camera.imageWidthInPixels <<
    //             " | Obj size (pixels): " << objectSizeInPixels << 
    //             " | Dist from camera (m): " << objectsDistanceFromCameraInMeters << 
    //             " | Sens size (mm): " << retVal << std::endl;
#endif

    return retVal;
}

float calculateDistanceWithDataStruct(  int objectSizeInPixels, 
                                        ros_drone_swarm_mocap::mocap_worker_data& procData){

    return calculateDistanceFull(   procData.camera.XfocalLengthInMillimeters, 
                                    procData.camera.objectsRealSizeInMeter, 
                                    procData.camera.imageWidthInPixels, 
                                    objectSizeInPixels, 
                                    procData.camera.XsensorSizeInMillimeters);
}

void saveDistancesToProcData(std::vector<cv::Vec3f> circles, ros_drone_swarm_mocap::mocap_worker_data& procData, const std::vector<double> inCircleLedDur){
    ros_drone_swarm_mocap::detected_ball_data bd;
    for( uint k = 0; k < circles.size(); k++ ){
        bd.image_plane_x        = circles[k][0];
        bd.image_plane_y        = circles[k][1];
        bd.image_plane_r        = circles[k][2];
        bd.distance_from_camera = calculateDistanceWithDataStruct(2*bd.image_plane_r, procData);
        bd.xangle               = calcucateAngleX(procData.camera.XFieldOfViewInAngles, procData.camera.imageWidthInPixels, bd.image_plane_x);
        bd.yangle               = calcucateAngleY(procData.camera.YFieldOfViewInAngles, procData.camera.imageHeightInPixels, bd.image_plane_y);
        // ROS_INFO("Circle - %d Center: (%d, %d) | Distance: %f | Angle x: %f | Angle y: %f", k,  
                    //  bd.image_plane_x, bd.image_plane_y,  bd.distance_from_camera,  bd.xangle, bd.yangle);
        // ROS_INFO("Sensorsize: %f\n", calculateSensorSize(2*bd.image_plane_r, 1.0, procData));
#ifdef DEBUG_EXTR_DATA_TO_FILE
#ifndef TESTING_HSV
        DEBUG_DA(ros::Time::now().toSec(), k, bd.distance_from_camera, bd.xangle, bd.yangle);
#endif
#endif
        if(!inCircleLedDur.empty())
            bd.id = get_id(inCircleLedDur[k]);
        else
            bd.id = 0;
    }
    procData.balls.push_back(bd);
}

void fixCenterRadius(std::vector<cv::Vec3f> &circles){
    static std::vector<cv::Vec3f> circles_buf;
    // FIXME Due to noise, for long ranges (short radius circles) increase diameter size
    // to improve range estimation  
    // for(int i=0; i<circles.size();i++){
    //     if (2*circles[i][2] < 100){
    //         circles[i][2] = circles[i][2]*1.1; 
    //     }
    //     else if (2*circles[i][2] < 50){
    //         circles[i][2] = circles[i][2]*1.15; 
    //     }
    //     else if (2*circles[i][2] < 30){
    //         circles[i][2] = circles[i][2]*1.2; 
    //     }
    //     else if (2*circles[i][2] < 15){
    //         circles[i][2] = circles[i][2]*1.3; 
    //     }
    // }
}
