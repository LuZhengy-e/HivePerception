#include<iostream>
#include<fstream>
#include<opencv2/core/core.hpp>
#include<pcl/conversions.h>
#include "Marker.h"
#include "KeyFrame.h"
#include "data.h"

using namespace HIVE_SLAM;

int main(int argc, char* argv[]){
    if(argc != 3){
        std::cerr << "Please input the bin dir and pcd dir" << std::endl;
    }

    const std::string input = argv[1];
    const std::string output = argv[2];
    
    return 0;
}
