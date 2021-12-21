/**
 * @file Optimizer.h
 * @author Zhengye (996227064@qq.com)
 * @brief Optimizer by g2o
 * @version 0.1.0
 * @date 2021-11-21
 **/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include<iostream>
#include<cmath>
#include<Eigen/Core>
#include<vector>
#include<jsoncpp/json/json.h>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Element.h"
#include "KeyFrame.h"
#include "Marker.h"

namespace HIVE_SLAM{

class PointElement;
class KeyFrame;
namespace optimizer{

class PointOptimizer{
public:
    /**
     * @brief BundleAdjustment by point and pos
     * @param[in] vpKFs                 Keyframe
     * @param[in] vvpPoints             all point
     * @param[in] nIterations           number of iterations
     * @param[in] bRobust               if use rbobust kernel function
     **/
    void static BundleAdjustment(const std::vector<KeyFrame*>& vpKF,
                                 const std::vector<std::vector<PointElement*> >& vvpPoints,
                                 const int nIterations,
                                 bool bRobust);
    
    /**
     * @brief BundleAdjustment by each type of point and pos
     * @param[in] vpKFs                 Keyframe
     * @param[in] vpPoints              each point type
     * @param[in] nIterations           number of iterations
     * @param[in] bRobust               if use rbobust kernel function
     **/
    void static TypeBundleAdjustment(const std::vector<KeyFrame*>& vpKF,
                                     const std::vector<PointElement*>& vpPoints,
                                     const int nIterations,
                                     bool bRobust);

    /**
     * @brief LocalBA
     * @param[in] vpKFs
     * @param[in] vpPoints
     * @param[in] nIterations
     * @param[in] bRobust
     **/
    void static LocalBundleAdjustment(const std::vector<KeyFrame*>& vpKF,
                                      const std::vector<PointElement*>& vpPoints,
                                      const int nIterations,
                                      bool bRobust);

};

};

class utils{
public:
    /**
     * @brief Triangulation one point and create a new point element 
     * @param[in] pKF1    KeyFrame1
     * @param[in] pKF2    KeyFrame2
     * @param[in] Posi    Pointi position2d
     * @param[in] pos_3D  pos 3d of point
     **/
    cv::Mat static Triangulation(KeyFrame* pKF1,
                                 KeyFrame* pKF2,
                                 const cv::Mat& Pos1,
                                 const cv::Mat& Pos2);

    /**
     * @brief project pixel coord to camera coord
     * @param[in] u pixel coord
     * @param[in] K inner parameters
     **/
    cv::Point2d static pixel2cam(const cv::Mat& u, 
                                 const cv::Mat& K);

    void static writeVisulizationJson(const std::string output_path,
                                      std::map<std::string, std::map<int, std::map<char, PointElement*> > > group_3D);

};

};


#endif