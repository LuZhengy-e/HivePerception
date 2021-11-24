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
#include<jsoncpp/json/json.h>

namespace HIVE_SLAM{

class NuScenes{
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

};


#endif