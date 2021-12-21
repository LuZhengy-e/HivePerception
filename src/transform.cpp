/**
 * @file transform.cpp
 * @author Zhengye (996227064@qq.com)
 * @brief Some common transform function of each coordinate, used for evaluation
 * @version 0.1.0
 * @date 2021-11-19
 **/

#include "transform.h"

namespace HIVE_SLAM{

CoordTranform::CoordTranform(){};

CoordTranform::CoordTranform(cv::Mat& K, 
                             cv::Mat& Tcl,
                             cv::Mat& Tcv)
{
    K.copyTo(_mK);
    Tcl.copyTo(_mTcl);
    Tcv.copyTo(_mTcv);
}

cv::Mat CoordTranform::Tlc(cv::Mat& point){
    return _mTcl * point;
}

cv::Mat CoordTranform::Tcp(const cv::Mat& point, const cv::Mat& Tcw){
    cv::Mat rotate_p = Tcw * point;
    double z = rotate_p.at<double>(2, 0);
    cv::Mat pixel = _mK * (rotate_p.rowRange(0, 3)) / z;

    return pixel.clone();
}

cv::Mat CoordTranform::Tlp(cv::Mat& point){
    cv::Mat camera_p = _mTcl * point;
    double z = camera_p.at<double>(2, 0);

    if(z <= 0){
        std::cerr << "cur point is behind the camera" << std::endl;
    }
    
    return _mK * (camera_p.rowRange(0, 3)) / z;
}

cv::Mat CoordTranform::getTcw(cv::Mat& Pos){
    cv::Mat Tvc;
    cv::invert(_mTcv, Tvc);

    return Tvc * Pos;
}

};