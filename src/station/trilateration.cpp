#include "station/trilateration.hpp"

trilatRet trilateration(const std::vector<anchor_data> anchors, Point &objectsPose){

    if (anchors.size() < 4){
        return NOT_ENOUGH_ANCHORS;
    }
    
    // for(int i=0;i<anchors.size();i++){
    //     std::cout << i << ". (" << anchors[i].pose.pos.x << "," << anchors[i].pose.pos.y << "," << anchors[i].pose.pos.z << ") - R: " << anchors[i].objectsDistance << std::endl; 
    // }
    // std::cout << std::endl;

    const int size = 4;//anchors.size();
    Eigen::Matrix <float, size, 4> A;
    A.setZero();

    A << 1, -2*anchors[0].pose.pos.x, -2*anchors[0].pose.pos.y, -2*anchors[0].pose.pos.z, 
        1, -2*anchors[1].pose.pos.x, -2*anchors[1].pose.pos.y, -2*anchors[1].pose.pos.z,
        1, -2*anchors[2].pose.pos.x, -2*anchors[2].pose.pos.y, -2*anchors[2].pose.pos.z,
        1, -2*anchors[3].pose.pos.x, -2*anchors[3].pose.pos.y, -2*anchors[3].pose.pos.z;

    // std::cout << A << std::endl;
    // std::cout << std::endl;

    Eigen::Matrix <float, size, 1> B;
    B.setZero();

    B << pow(anchors[0].objectsDistance,2) - pow(anchors[0].pose.pos.x,2) - pow(anchors[0].pose.pos.y,2) - pow(anchors[0].pose.pos.z,2),
        pow(anchors[1].objectsDistance,2) - pow(anchors[1].pose.pos.x,2) - pow(anchors[1].pose.pos.y,2) - pow(anchors[1].pose.pos.z,2),
        pow(anchors[2].objectsDistance,2) - pow(anchors[2].pose.pos.x,2) - pow(anchors[2].pose.pos.y,2) - pow(anchors[2].pose.pos.z,2),
        pow(anchors[3].objectsDistance,2) - pow(anchors[3].pose.pos.x,2) - pow(anchors[3].pose.pos.y,2) - pow(anchors[3].pose.pos.z,2);

    // std::cout << B << std::endl;
    // std::cout << std::endl;
    // std::cout << std::endl;

    // Eigen::Matrix <float, 4, 1> x = A.colPivHouseholderQr().solve(B);
    Eigen::Matrix <float, 4, 1> x = (A.transpose() * A).ldlt().solve(A.transpose() * B);
    objectsPose.pos.x = x[1];
    objectsPose.pos.y = x[2];
    objectsPose.pos.z = x[3]; //(x[3] < 0) ? 0 : x[3];
    // std::cout << "Estimation\n" << x << std::endl << std::endl;

    return SUCCESS;
}