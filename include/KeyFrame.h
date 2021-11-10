/**
 * @file KeyFrame.h
 * @author Zhengye (996227064@qq.com)
 * @brief KeyFrame
 * @version 0.1.0
 * @date 2021-11-09
 **/

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include<iostream>
#include<fstream>
#include<opencv2/core/core.hpp>
#include "Frame.h"
#include "Element.h"

namespace HIVE_SLAM{

class PointElement;
class KeyFrame{
public:
    /**
     * @brief Create a keyframe
     * @param[in] img The origin image of this keyframe
     * @param[in] Tcw The pos of this keyframe
     * @param[in] K The internal parameters matrix
     **/
    explicit KeyFrame(const cv::Mat &img, const cv::Mat &Tcw, const cv::Mat &K);

    /**
     * @brief Project the point to this keyframe
     * @param[in] point The point need to be projected
     **/
    cv::Mat ProjectPoint(PointElement* Point);

protected:
    cv::Mat mTcw;
    cv::Mat mK;
};

};

#endif