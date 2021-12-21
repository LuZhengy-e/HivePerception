/**
 * @file Marker.h
 * @author Zhengye (996227064@qq.com)
 * @brief Poles
 * @version 0.1.0
 * @date 2021-12-17
 **/

#include "Pole.h"

namespace HIVE_SLAM{

void Pole::setID(){
    mnId = poleID++;
}

void Pole::setPosType(const char& postype){
    mpostype = postype;
}

void Pole::addFriends(const long unsigned int& friendId){
    msfriends.insert(friendId);
}

};