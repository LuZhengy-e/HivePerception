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
        std::cerr << "Please input the datasets type and root dir" << std::endl;
    }

    const std::string data_type = argv[1];
    const std::string root_dir = argv[2];

    if(data_type == "kitti"){
        std::cout << "start to create kitti datasets" << std::endl;
        Kitti datasets = Kitti(root_dir);
        datasets.read_results("/result/"); // should write "/" manually
    }
    else if(data_type == "nuscenes"){
        std::cout << "start to create nuscenes datasets" << std::endl;
    }
    
    return 0;
}
