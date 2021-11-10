/**
 * @file KeyFrame.h
 * @author Zhengye (996227064@qq.com)
 * @brief KeyFrame
 * @version 0.1.0
 * @date 2021-11-10
 **/

#include "KeyFrame.h"

namespace HIVE_SLAM{

KeyFrame::KeyFrame(const cv::Mat &img, 
                   const cv::Mat &Tcw,
                   const cv::Mat &K)
{
    Tcw.copyTo(mTcw);
    K.copyTo(mK);
}

cv::Mat KeyFrame::ProjectPoint(PointElement* point){
    cv::Mat P = point->getWorldPos();
    cv::Mat P_homogeneous = cv::Mat::ones(4, 1, CV_64F);
    P.copyTo(P_homogeneous.rowRange(0, 3));
    double z = P.at<double>(2, 0);

    cv::Mat rotate_p = mTcw * P_homogeneous / z;

    return mK * (rotate_p.rowRange(0, 3));
}

}