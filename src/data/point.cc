
#include "point.h"

namespace modules_vins
{


KeyPoint::KeyPoint():
id_(-1){
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;
}

KeyPoint::KeyPoint(const cv::KeyPoint &kp):
cv_keypoint_(kp), id_(-1), point2d_(kp.pt){
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;

}

void KeyPoint::setId(int id){
    this->id_ = id;
}

void KeyPoint::setCVKeyPoint(const cv::KeyPoint &kp){
    this->cv_keypoint_ = kp;
    this->point2d_ = kp.pt;

}

void KeyPoint::set3DKeyPoint(const cv::Point3f point3d){
    this->point3d_ = point3d;
}


void KeyPoint::setKeyPointPosition(const double x, const double y){
    this->point2d_.x = x;
    this->point2d_.y = y;
}

void KeyPoint::setMatchInTime(const cv::DMatch &match_in_time){
    this->match_in_time_ = match_in_time;
}



// void KeyPoint::setMatchInFrame(const cv::DMatch &match_in_frame){
//     this->match_in_frame_ = match_in_frame;
// }

    
} // namespace modules_vins





