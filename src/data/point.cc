
#include "point.h"

namespace modules_vins
{


KeyPoint::KeyPoint():
id_(-1){
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;
}

KeyPoint::KeyPoint(const cv::KeyPoint &kp):
cv_keypoint_(kp), id_(-1), x_(kp.pt.x), y_(kp.pt.y){
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;

}

void KeyPoint::setId(int id){
    this->id_ = id;
}

void KeyPoint::setCVKeyPoint(const cv::KeyPoint &kp){
    this->cv_keypoint_ = kp;
    this->x_ = kp.pt.x;
    this->y_ = kp.pt.y;
}


void KeyPoint::setKeyPointPosition(const double x, const double y){
    this->x_ = x;
    this->y_ = y;
}

void KeyPoint::setMatchInTime(const cv::DMatch &match_in_time){
    this->match_in_time_ = match_in_time;
}



// void KeyPoint::setMatchInFrame(const cv::DMatch &match_in_frame){
//     this->match_in_frame_ = match_in_frame;
// }

    
} // namespace modules_vins





