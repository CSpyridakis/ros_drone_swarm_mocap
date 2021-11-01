#include "station/trilateration.hpp"

trilRet trilateration(std::vector<AnchorData> anchors, Point objectLocation){
    Point ballCoordinates = {0.0,0.0,0.0};

    if (anchors.size() < 4){
        return NOT_ENOUGH_ANCHORS;
    }
    return SUCCESS;
}