/**
 * @file Element.h
 * @author Zhengye (996227064@qq.com)
 * @brief Abstract class instance of each elements 
 * @version 0.1.0
 * @date 2021-11-09
 **/

#ifndef ELEMENT_H
#define ELEMENT_H

#include<map>
#include<set>
#include "KeyFrame.h"
#include<opencv2/core/core.hpp>
#include<Eigen/Core>

namespace HIVE_SLAM{

class KeyFrame;
class PointElement{
public:
    /**
     * @brief Create a point element
     * @param[in] Pos The pos in global coord
     * @param[in] pFirstKF The First keyframe that observe this point
     * @param[in] on_plane If this point type is on a fixed plane
     **/
    explicit PointElement(const cv::Mat &Pos, KeyFrame* pFirstKF, bool on_plane);

    /**
     * @brief Set global id of each kind of element, need to overwrite
     **/
    virtual void setID() = 0;

    /**
     * @brief Set type of this point
     * @param[in] type
     **/
    void setType(std::string type);

    /**
     * @brief Add new observation relation
     * @param[in] pKF The keyframe that observe this point
     * @param[in] idx the index of this point in the keyframe
     **/
    void addObservations(KeyFrame* pKF, int idx);

    /**
     * @brief Delete observation relation
     * @param[in] pKF the keyframe need to be deleted
     **/
    void deleteObservations(KeyFrame* pKF);

    std::map<KeyFrame*, int> getObservations() const;

    /**
     * @brief Get world pos
     * @return World pos
     **/
    cv::Mat getWorldPos() const;

    /**
     * @brief Set or update world pos
     * @param[in] Pos Global world pos
     **/
    void setWorldPos(const cv::Mat &Pos);

    /**
     * @brief Get projection of this point to the keyframe
     * @param[in] KeyFrame The keyframe projected to
     * @return The pixel pos of this point in the keyframe
     **/
    cv::Mat getProjection(KeyFrame* pKF);

    Eigen::Matrix<double,3,1> getVectorPos() const;

    long unsigned int getId() const;

    std::string getType() const;

    
protected:
    cv::Mat mWorldPos; // The global pos of this 
    std::map<KeyFrame*, int> mObversations; // <The keyframe that can observe this point, the index of this point in the keyframe>
    bool mbon_plane;
    unsigned int mnObs;
    long unsigned int mnId; // The global id of PointElement
    std::string mType; 
    
};

};

#endif