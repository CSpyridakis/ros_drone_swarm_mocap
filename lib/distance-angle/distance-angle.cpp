/**
 * 
 */
float calculateAngle(int fov, int pixelLenght, int NthPixel, bool yaxis = false){
    float angle = (float)( (std::abs( (float)pixelLenght/2 - NthPixel) * fov ) / pixelLenght);  ///< Calculate angle

    if (yaxis) return (float)(NthPixel >= pixelLenght/2 ) ? -angle : angle  ;       // Calculate sign (for y axis ymiddle < 0 -> positive angles)
    return (float)(NthPixel >= pixelLenght/2 ) ? angle : -angle  ;                  // Calculate sign (for x axis xmiddle < 0 -> negative angles)
}

/**
 */
float calculateSensorsSizeFull(float focalLengthInMillimeter, float objectsRealSizeInMeter, 
                            int imageSizeInPixels, int objectsSizeInPixels, float objectsDistanceFromCameraInMeters){
    
    // ROS_INFO("\nfocalLengthInMillimeter: %f  \nobjectsRealSizeInMeter: %f \nimageSizeInPixels: %d \nobjectsSizeInPixels: %d \nobjectsDistanceFromCameraInMeters:%f\n", focalLengthInMillimeter, objectsRealSizeInMeter, imageSizeInPixels, objectsSizeInPixels, objectsDistanceFromCameraInMeters);

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / ((float)objectsSizeInPixels * objectsDistanceFromCameraInMeters) );
}

/**
 */
float calculateSensorSize(int objectSizeInPixels, float objectsDistanceFromCameraInMeters, 
                        ros_drone_swarm_mocap::mocap_worker_data& procData){
    return calculateSensorsSizeFull(procData.XfocalLengthInMillimeters, procData.objectsRealSizeInMeter, procData.imageWidthInPixels,
                            objectSizeInPixels, objectsDistanceFromCameraInMeters);
}

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
                            int imageSizeInPixels, int objectsSizeInPixels, float sensorSizeInMillileter ){

    // ROS_INFO("\nfocalLengthInMillimeter: %f  \nobjectsRealSizeInMeter: %f \nimageSizeInPixels: %d \nobjectsSizeInPixels: %d \nsensorSizeInMillimeter:%f\n", focalLengthInMillimeter, objectsRealSizeInMeter, imageSizeInPixels, objectsSizeInPixels, sensorSizeInMillileter);

    return (float)( (focalLengthInMillimeter * objectsRealSizeInMeter * (float)imageSizeInPixels) / ((float)objectsSizeInPixels * sensorSizeInMillileter) );
}