#include <iostream>
#include "distance-angle.hpp"

float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis = false){
    float angle = (float)( (std::abs( (float)pixelLenght/2 - NthPixel ) * fov ) / pixelLenght);

    // Calculate sign (for y axis ymiddle < 0 -> positive angles)
    if (yaxis) return (float)(NthPixel >= pixelLenght/2 ) ? -angle : angle  ;       
    // Calculate sign (for x axis xmiddle < 0 -> negative angles)
    return (float)(NthPixel >= pixelLenght/2 ) ? angle : -angle  ;                 
}

float calcucateAngleX(int fov, int pixelLenght, int NthPixel){
    return calculateAngle(fov, pixelLenght, NthPixel, false);
}

float calcucateAngleY(int fov, int pixelLenght, int NthPixel){
    return calculateAngle(fov, pixelLenght, NthPixel, true);
}

// -----------------------------------------------------------------------------------------------------

float calculateSensorsSizeFull( float focalLengthInMillimeter, 
                                float objectsRealSizeInMeter, 
                                int imageSizeInPixels, 
                                int objectsSizeInPixels, 
                                float objectsDistanceFromCameraInMeters){

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / 
                        ((float)objectsSizeInPixels * objectsDistanceFromCameraInMeters) );
}

float calculateDistanceFull(float focalLengthInMillimeter, 
                            float objectsRealSizeInMeter, 
                            int imageSizeInPixels, 
                            int objectsSizeInPixels, 
                            float sensorSizeInMillileter){

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / 
                        ((float)objectsSizeInPixels * sensorSizeInMillileter) );
}