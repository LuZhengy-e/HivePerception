/**
 * @file Element.h
 * @author Zhengye (996227064@qq.com)
 * @version 0.1.0
 * @date 2021-11-10
 **/

#include "Element.h"
#include<iostream>

namespace HIVE_SLAM{

PointElement::PointElement(const cv::Mat &Pos, 
                           KeyFrame* pFirstKF, 
                           bool on_plane): mbon_plane(on_plane)
{
    Pos.copyTo(mWorldPos);
    mnObs = 1;
}

void PointElement::setType(std::string type){
    mType = type;
}

void PointElement::addObservations(KeyFrame* pKF, int idx){
    if(mObversations.count(pKF)){
        return;
    }
    mObversations[pKF] = idx;
    mnObs += 1;
}

void PointElement::deleteObservations(KeyFrame* pKF){
    if(mObversations.count(pKF)){
        mObversations.erase(pKF);
        mnObs -= 1;
    }
}

cv::Mat PointElement::getWorldPos() const{
    cv::Mat tmp_P;
    mWorldPos.copyTo(tmp_P);
    return tmp_P;
}

void PointElement::setWorldPos(const cv::Mat &Pos){
    Pos.copyTo(mWorldPos);
}

cv::Mat PointElement::getProjection(KeyFrame* pKF){
    if(mObversations.count(pKF)){
        return pKF->ProjectPoint(this);
    }
}

};