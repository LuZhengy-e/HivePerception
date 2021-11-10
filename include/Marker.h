/**
 * @file Marker.h
 * @author Zhengye (996227064@qq.com)
 * @brief Markers
 * @version 0.1.0
 * @date 2021-11-09
 **/
#ifndef MARKER_H
#define MARKER_H

#include "Element.h"
#include "KeyFrame.h"

namespace HIVE_SLAM{

static long unsigned int markerID = 0;

class KeyFrame;
class PointElement;
class Marker: public PointElement{
    virtual void setID();
};

};

#endif