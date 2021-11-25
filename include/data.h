/**
 * @file data.h
 * @author Zhengye (996227064@qq.com)
 * @brief Read datasets tools
 * @version 0.1.0
 * @date 2021-11-20
 **/

#ifndef DATA_H
#define DATA_H

#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<jsoncpp/json/json.h>
#include<pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace HIVE_SLAM{

class NuScenes{
public:
    /**
     * @brief Empty construct function
     **/
    NuScenes();

    /**
     * @brief Construct by input dir name
     * @param[in] filename input dir of nuscenes datasets
     **/
    NuScenes(const std::string& filename);

    void read_from_file(const std::string& filename);
};

class Kitti{
public:
    /**
     * @brief Empty construct function
     **/
    Kitti();

    /**
     * @brief Construct by input dir name
     * @param[in] filename input dir of nuscenes datasets
    **/
    Kitti(const std::string& filename);

    void read_from_file(const std::string& filename);

    static void bin2pcd(const std::string& in_file, const std::string& out_file);

};

};


#endif