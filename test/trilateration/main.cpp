#include <iostream>

#include "station/trilateration.hpp"

#define REAL 0
#define ESTIMATED 1

float node1[][] = {{1.820, 1.539, 2.488, 2.471, 1.510, 1.593, 1.473, 1.627, 1.914, 2.129, 2.209, 2.138, 1.837},
                    {1.886, 1.248, 2.273, 2.333, 1.248, 1.641, 1.502, 1.611, 1.847, 2.061, 2.162, 2.162, 1.847}};

float node2[][] = {{1.847, 1.560, 1.507, 2.519, 2.522, 2.146, 1.927, 1.643, 1.478, 1.610, 1.919, 2.168, 2.225},
                    {1.927, 1.343, 1.198, 2.333, 2.396, 2.162, 1.970, 1.672, 1.528, 1.611, 1.886, 2.110, 2.273}};

float node3[][] = {{1.765, 2.475, 1.514, 1.469, 2.396, 2.040, 2.157, 2.071, 1.831, 1.568, 1.427, 1.547, 1.836},
                    {1.809, 2.333, 1.231, 1.182, 2.162, 2.014, 2.162, 2.110, 1.809, 1.583, 1.453, 1.528, 1.773}};

float node4[][] = {{1.839, 2.510, 2.499, 1.515, 1.532, 1.617, 1.870, 2.139, 2.223, 2.155, 1.877, 1.591, 1.488},
                    {1.886, 2.273, 2.396, 1.323, 1.182, 1.555, 1.773, 2.061, 2.110, 2.110, 1.886, 1.611, 1.528}};


int main(int argv, char * argc[]){
    std::cout << "Hello " << std::endl;
}
