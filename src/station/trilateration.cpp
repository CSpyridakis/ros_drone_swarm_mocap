#include "station/trilateration.hpp"

#define DISPLAY_MAT(x)              { std::cout << x << std::endl << std::endl; }
#define DISPLAY_ESTIMATION()        { std::cout << "Estimation:\n" << x << std::endl << std::endl; }
#define DISPLAY_ANCHORS_POSITIONS() {\
    for(int i=0;i<anchors.size();i++){                                                                          \
        std::cout << i << "." <<                                                                                \
        "(" << anchors[i].pose.pos.x << "," << anchors[i].pose.pos.y << "," << anchors[i].pose.pos.z << ")" <<  \ 
        "- R: " << anchors[i].objectsDistance << std::endl;                                                     \ 
    }                                                                                                           \
    std::cout << std::endl;}                                                 

trilatRet trilateration(const std::vector<anchor_data> anchors, Point &objectsPose){
    if (anchors.size() < 4)
        return NOT_ENOUGH_ANCHORS;

    if (anchors.size() + 1 > MAX_NUM_OF_ACHORS)
        return TOO_MANY_ANCHORS;
    
    // DISPLAY_ANCHORS_POSITIONS();

    // FIXME you need to change trilateration implementation to utilize more than 4 anchors!
    const int size = anchors.size();
    
    // Create matrix A, make sure that is empty and only then set its values
    Eigen::Matrix <float, size, 4> A;
    A.setZero();
    A   <<  1, -2*anchors[0].pose.pos.x, -2*anchors[0].pose.pos.y, -2*anchors[0].pose.pos.z, 
            1, -2*anchors[1].pose.pos.x, -2*anchors[1].pose.pos.y, -2*anchors[1].pose.pos.z,
            1, -2*anchors[2].pose.pos.x, -2*anchors[2].pose.pos.y, -2*anchors[2].pose.pos.z,
            1, -2*anchors[3].pose.pos.x, -2*anchors[3].pose.pos.y, -2*anchors[3].pose.pos.z;
    // PRINT_MAT(A);

    // Do the same with matrix B
    Eigen::Matrix <float, size, 1> B;
    B.setZero();
    B   <<  pow(anchors[0].objectsDistance,2) - pow(anchors[0].pose.pos.x,2) - pow(anchors[0].pose.pos.y,2) - pow(anchors[0].pose.pos.z,2),
            pow(anchors[1].objectsDistance,2) - pow(anchors[1].pose.pos.x,2) - pow(anchors[1].pose.pos.y,2) - pow(anchors[1].pose.pos.z,2),
            pow(anchors[2].objectsDistance,2) - pow(anchors[2].pose.pos.x,2) - pow(anchors[2].pose.pos.y,2) - pow(anchors[2].pose.pos.z,2),
            pow(anchors[3].objectsDistance,2) - pow(anchors[3].pose.pos.x,2) - pow(anchors[3].pose.pos.y,2) - pow(anchors[3].pose.pos.z,2);
    // PRINT_MAT(B);
    
    // Calculate X matrix by solving the linear system
    // Eigen::Matrix <float, 4, 1> x = A.colPivHouseholderQr().solve(B);
    Eigen::Matrix <float, 4, 1> x = (A.transpose() * A).ldlt().solve(A.transpose() * B);
    objectsPose.pos.x = x[1];
    objectsPose.pos.y = x[2];
    objectsPose.pos.z = x[3]; //(x[3] < 0) ? 0 : x[3];
    
    //DISPLAY_ESTIMATION();

    return SUCCESS;
}