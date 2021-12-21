/**
 * @file Pole.h
 * @author Zhengye (996227064@qq.com)
 * @brief Pole(Two key point and a line equation in 3d)
 * @version 0.1.0
 * @date 2021-11-09
 **/



#ifndef POLE_H
#define POLE_H

#include<iostream>
#include<fstream>
#include<opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "Element.h"


namespace HIVE_SLAM{

static long unsigned int poleID = 0;

class KeyFrame;
class PointElement;
class Pole: public PointElement{
public:
    using PointElement::PointElement;
    
    /**
     * @brief set global markerId
     **/
    virtual void setID();

    /**
     * @brief setType: 0: left up, 1: right up, 2: right bottom, 3: left bottom
     * @param[in] postype 
     **/
    void setPosType(const char& postype);

    /**
     * @brief add other marker point id in mvfriends
     * @param[in] friendId
     **/
    void addFriends(const long unsigned int& friendId);

private:
    char mpostype;
    std::set<long unsigned int> msfriends; // marker points that belong to same mark

};

};

#endif