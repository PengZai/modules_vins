
#include "point.h"

namespace modules_vins
{


KeyPoint::KeyPoint():
cv::KeyPoint(), id_(-1){
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;
}

KeyPoint::KeyPoint(const cv::KeyPoint &kp):
cv::KeyPoint(kp), id_(-1){
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;

}

void KeyPoint::setId(int id){
    this->id_ = id;
}

void KeyPoint::setMatchInTime(const cv::DMatch &match_in_time){
    this->match_in_time_ = match_in_time;
}

// void KeyPoint::setMatchInFrame(const cv::DMatch &match_in_frame){
//     this->match_in_frame_ = match_in_frame;
// }

    
} // namespace modules_vins





