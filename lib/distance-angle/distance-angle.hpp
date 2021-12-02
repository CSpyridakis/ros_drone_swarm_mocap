#ifndef DISTANCE_ANGLE_HPP
#define DISTANCE_ANGLE_HPP

typedef struct angle_data{
    int fov;
    int pixelLenght;
    int NthPixel;
}angle_data;

typedef struct range_data{
    float focalLengthInMillimeter;
    float objectsRealSizeInMeter; 
    int imageSizeInPixels; 
    int objectsSizeInPixels; 
    float objectsDistanceFromCameraInMeters;
} range_data;

/**
 * 
 */
float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis = false);



/**
 */
float calculateSensorsSizeFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float objectsDistanceFromCameraInMeters);


/**
 * \brief
 * 
 * \param focalLengthInMillimeter
 * \param objectsRealSizeInMeter
 * \param imageSizeInPixels
 * \param objectsSizeInPixels
 * \param sensorSizeInMillimeter
 * 
 * \return 
 */
float calculateDistanceFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter);


float calculateDistanceUsingStruct(range_data data);
   

#endif