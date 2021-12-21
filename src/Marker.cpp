/**
 * @file Marker.h
 * @author Zhengye (996227064@qq.com)
 * @brief Markers
 * @version 0.1.0
 * @date 2021-11-10
 **/

#include "Marker.h"

namespace HIVE_SLAM{

void Marker::setID(){
    mnId = markerID++;
}

void Marker::setPosType(const char& postype){
    mpostype = postype;
}

void Marker::addFriends(const long unsigned int& friendId){
    msfriends.insert(friendId);
}

};