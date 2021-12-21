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
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<jsoncpp/json/json.h>
#include<pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "KeyFrame.h"
#include "Marker.h"
#include "Pole.h"
#include "transform.h"
#include "Optimizer.h"

namespace HIVE_SLAM{

void getFile(const std::string& dirname, std::vector<std::string>& files);

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
     * @brief Construct by input dir name
     * @param[in] filename input dir of kitti datasets
    **/
    explicit Kitti(const std::string& filename);

    /**
     * @brief convert .bin velodyne file to .pcd file
     * @param[in] in_file input dir of bin file
     * @param[in] out_file output dir of pic file
     **/
    static void bin2pcd(const std::string& in_file, const std::string& out_file);

    /**
     * @brief read json label to Keyframes, and triangulation by keyframe
     * @param[in] in_file input idr of json file
     **/
    void read_results(const std::string& in_file);

private:
    std::string mrootDir;
    std::map<long unsigned int, KeyFrame*> mmptraj;
    std::map<long unsigned int, PointElement*> mmpmarkers;
    std::map<long unsigned int, PointElement*> mmpoles;
    CoordTranform mtrans_handler;

};

};


#endif