#include "station/trilateration.hpp"

trilatRet trilateration(const std::vector<anchor_data> anchors, Point &objectsPose){
    Point objPose = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    if (anchors.size() < 4){
        return NOT_ENOUGH_ANCHORS;
    }
    return SUCCESS;
}