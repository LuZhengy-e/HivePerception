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
    mnId = KeyFrameId++;
}

cv::Mat KeyFrame::ProjectPoint(PointElement* point){
    cv::Mat P = point->getWorldPos();
    cv::Mat P_homogeneous = cv::Mat::ones(4, 1, CV_64F);
    P.copyTo(P_homogeneous.rowRange(0, 3));
    double z = P.at<double>(2, 0);

    cv::Mat rotate_p = mTcw * P_homogeneous / z;
    cv::Mat pixel = mK * (rotate_p.rowRange(0, 3));

    return pixel.clone();
}

void KeyFrame::setPos(const cv::Mat& Tcw){
    Tcw.copyTo(mTcw);
}

cv::Mat KeyFrame::getPos(){
    return mTcw.clone();
}

g2o::SE3Quat KeyFrame::getSE3Pos() const{
    //首先将旋转矩阵提取出来
    Eigen::Matrix<double,3,3> R;
    R << mTcw.at<double>(0,0), mTcw.at<double>(0,1), mTcw.at<double>(0,2),
         mTcw.at<double>(1,0), mTcw.at<double>(1,1), mTcw.at<double>(1,2),
         mTcw.at<double>(2,0), mTcw.at<double>(2,1), mTcw.at<double>(2,2);

	//然后将平移向量提取出来
    Eigen::Matrix<double,3,1> t(mTcw.at<double>(0,3), mTcw.at<double>(1,3), mTcw.at<double>(2,3));

    return g2o::SE3Quat(R, t);
}
unsigned long int KeyFrame::getId() const{
    return mnId;
}

PointElement* KeyFrame::getPoint(const std::string& type, const int id){
    return mvPoints[type][id];
}

cv::Mat KeyFrame::getInner() const{
    return mK.clone();
}

}