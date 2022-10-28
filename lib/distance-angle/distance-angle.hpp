#ifndef DISTANCE_ANGLE_HPP
#define DISTANCE_ANGLE_HPP

/**
 * This struct holds 
*/
typedef struct angle_data{
    int fov;                //< contains
    int pixelLenght;        //< contains
    int NthPixel;           //< contains
    bool yaxis;             //< contains
}angle_data;


/**
 * This struct holds 
*/
typedef struct range_data{
    float focalLengthInMillimeter;              //< contains
    float objectsRealSizeInMeter;               //< contains
    int imageSizeInPixels;                      //< contains
    int objectsSizeInPixels;                    //< contains
    float objectsDistanceFromCameraInMeters;    //< contains
    float sensorSizeInMillileter;               //< contains
} range_data;

/**
 * @brief
 * 
 * @param[in] fov
 * @param[in] pixelLenght
 * @param[in] NthPixel
 * @param[in] yaxis
 * 
 * @return 
 */
float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis);

/**
 * @brief
 * 
 * @param[in] fov
 * @param[in] pixelLenght
 * @param[in] NthPixel
 * 
 * @return 
 */
float calcucateAngleX(int fov, int pixelLenght, int NthPixel);

/**
 * @brief
 * 
 * @param[in] fov
 * @param[in] pixelLenght
 * @param[in] NthPixel
 * 
 * @return 
 */
float calcucateAngleY(int fov, int pixelLenght, int NthPixel);

/**
 * @brief
 * 
 * @param[in] focalLengthInMillimeter
 * @param[in] objectsRealSizeInMeter
 * @param[in] imageSizeInPixels
 * @param[in] objectsSizeInPixels
 * @param[in] objectsDistanceFromCameraInMeters
 * 
 * @return 
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
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillimeter);

#endif