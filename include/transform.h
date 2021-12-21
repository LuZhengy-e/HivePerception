/**
 * @file transform.h
 * @author Zhengye (996227064@qq.com)
 * @brief Some common transform function of each coordinate, used for evaluation or data read
 * @version 0.1.0
 * @date 2021-11-19
 **/

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include<opencv2/core/core.hpp>
#include<iostream>
#include<fstream>

namespace HIVE_SLAM{

class CoordTranform{
public:
    CoordTranform();
    /**
     * @brief construct function
     * @param[in] K inner parameter
     * @param[in] Tcl Transform matrix from camera to lidar
     * @param[in] Tcv Transform matrix from camera to vehicle 
     **/
    CoordTranform(cv::Mat& K, cv::Mat& Tcl, cv::Mat& Tcv);

    /**
     * @brief give a lidar point, transform it to camera coord
     * @param[in] point point in lidar coord
     **/
    cv::Mat Tlc(cv::Mat& point);

    /**
     * @brief give a 3d point, transform it to pixel coord
     * @param[in] point point in world coord 
     **/
    cv::Mat Tcp(const cv::Mat& point, const cv::Mat& Tcw);

    /**
     * @brief give a lidar point, transform it to pixel coord
     * @param[in] point point in lidar coord 
     **/
    cv::Mat Tlp(cv::Mat& point);

    /**
     * @brief give a vehicle pos, get camera pos
     * @param[in] Pos vehicle in world coord 
     **/
    cv::Mat getTcw(cv::Mat& Pos);

private:
    cv::Mat _mK;
    cv::Mat _mTcl;
    cv::Mat _mTcv;

};

};



#endif