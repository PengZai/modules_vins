
#include "point.h"

namespace modules_vins
{



int MapPoint::id_counter_ = -1;


MapPoint::MapPoint():
id_(++MapPoint::id_counter_)
{

};

MapPoint::MapPoint(const cv::Point3f pt):
id_(++MapPoint::id_counter_), pt_(pt)
{

};

void MapPoint::setPosition(const cv::Point3f pt){
    this->pt_ = pt;
}



int KeyPoint::id_counter_ = -1;



KeyPoint::KeyPoint():
id_(++KeyPoint::id_counter_),
map_point_ptr_(nullptr), 
next_keypoint_in_time_(nullptr), 
prev_keypoint_in_time_(nullptr)
{
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;
}

KeyPoint::KeyPoint(const cv::KeyPoint kp):
id_(++KeyPoint::id_counter_), 
pt_(kp.pt), 
cv_keypoint_(kp), 
map_point_ptr_(nullptr), 
next_keypoint_in_time_(nullptr), 
prev_keypoint_in_time_(nullptr)
{
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = id_;
    id_counter_++;

}



void KeyPoint::setCVKeyPoint(const cv::KeyPoint &kp){
    this->cv_keypoint_ = kp;
    this->pt_ = kp.pt;

}

void KeyPoint::setNextKeyPointInTime(const std::shared_ptr<KeyPoint> &next_keypoint_in_time){
    this->next_keypoint_in_time_ = next_keypoint_in_time;
}
void KeyPoint::setPrevKeyPointInTime(const std::shared_ptr<KeyPoint> &prev_keypoint_in_time){
    this->prev_keypoint_in_time_ = prev_keypoint_in_time;
}



void KeyPoint::setMapPointPtr(const std::shared_ptr<MapPoint> &map_point_ptr){
    this->map_point_ptr_ = map_point_ptr;
}


void KeyPoint::setKeyPointPosition(const double x, const double y){
    this->pt_.x = x;
    this->pt_.y = y;
}

void KeyPoint::setMatchInTime(const cv::DMatch &match_in_time){
    this->match_in_time_ = match_in_time;
}



void KeyPoint::setMatchInFrame(const cv::DMatch &match_in_frame){
    this->match_in_frame_ = match_in_frame;
}

    
} // namespace modules_vins





