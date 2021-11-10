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
#include "Marker.h"
#include "Pole.h"
#include "RoadLine.h"
#include "RoadPlane.h"
#include "Signal.h"

namespace HIVE_SLAM{

class PointElement;
class KeyFrame{
public:
    KeyFrame();
    KeyFrame(const cv::Mat &img, const cv::Mat &Tcw, const cv::Mat &K);
    cv::Mat ProjectPoint(PointElement* Point);

protected:
    cv::Mat mTcw;
    cv::Mat mK;
};

};

#endif