#include "station/trilateration.hpp"

trilatRet trilateration(const std::vector<anchor_data> anchors, Point &objectsPose){
    Point objPose = {.pos = {0.0, 0.0, 0.0}, .ori = {0.0, 0.0, 0.0}};

    if (anchors.size() < 4){
        return NOT_ENOUGH_ANCHORS;
    }
    return SUCCESS;
}