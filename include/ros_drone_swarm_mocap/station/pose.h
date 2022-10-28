#ifndef POSE_H
#define POSE_H

/**
 *  Use this struct to hold for an object its coordinates (based on cartesian coordinate system), in three-dimentional space 
*/
typedef struct Position{
    double x;           ///< contains the X-axis coordinate value
    double y;           ///< contains the Y-axis coordinate value
    double z;           ///< contains the Z-axis coordinate value
} Position;

/**
 * Use this struct to hold angular position (or direction) of an object that is placed in three-dimentional space
 */
typedef struct Orientation{
    double roll;        ///< contains X-axis angle (mnemonic rule: roll like a cat)
    double pitch;       ///< contains Y-axis angle (mnemonic rule: pitcher)
    double yaw;         ///< contains Z-axis angle (mnemonic rule: door yaw)
} Orientation;

/** 
 * Use this struct to hold pose of an object in three-dimentional space (both position and orientation)
*/
typedef struct Point{
    Position pos;       ///< contains object coordinates see @ref Position struct
    Orientation ori;    ///< contains object direction see @ref Orientation struct
} Point;

#endif //POSE_H