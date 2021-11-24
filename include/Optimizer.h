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

#include "Element.h"
#include "KeyFrame.h"

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


};

};

};


#endif