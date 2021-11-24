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

std::map<KeyFrame*, int> PointElement::getObservations() const{
    return mObversations;
}

cv::Mat PointElement::getWorldPos() const{
    return mWorldPos.clone();
}

void PointElement::setWorldPos(const cv::Mat &Pos){
    Pos.copyTo(mWorldPos);
}

cv::Mat PointElement::getProjection(KeyFrame* pKF){
    if(mObversations.count(pKF)){
        return pKF->ProjectPoint(this);
    }
}

Eigen::Matrix<double, 3, 1> PointElement::getVectorPos() const{
    Eigen::Matrix<double,3,1> v;
    v << mWorldPos.at<double>(0), mWorldPos.at<double>(1), mWorldPos.at<double>(2);

    return v;
}

long unsigned int PointElement::getId() const{
    return mnId;
}

std::string PointElement::getType() const{
    return mType;
}

};