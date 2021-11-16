#include<iostream>
#include<fstream>
#include<opencv2/core/core.hpp>
#include "Marker.h"
#include "KeyFrame.h"

using namespace HIVE_SLAM;

int main(int argc, char* argv[]){
    cv::Mat Tcw;
    cv::Mat Pos;

    if(argc != 2){
        std::cerr << "Please input root path of datasets" << std::endl;
    }

    const std::string root_path = argv[1];
    const std::string velodyne_data = "/velodyne_txt/000000.txt";
    std::string velodyne_path = root_path + velodyne_data;

    std::ifstream veloFile;
    veloFile.open(velodyne_path.c_str());
    if(!veloFile.is_open()){
        std::cerr << "Can't open this file: " << velodyne_path << std::endl;
    }

    std::string point;
    while(std::getline(veloFile, point)){
        std::cout << "point is: " << point << std::endl;
    }

    return 0;
}