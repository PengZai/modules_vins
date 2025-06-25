
#include "point.h"

namespace modules_vins
{



int MapPoint::id_counter_ = -1;


MapPoint::MapPoint():
id_(++MapPoint::id_counter_),
timestamp_(-1),
pt3d_(0,0,0),
bgr_(0,0,0)
{

};

MapPoint::MapPoint(const Eigen::Vector3d &pt3d):
MapPoint()
{
    this->pt3d_ = pt3d;
};

void MapPoint::setPosition(const Eigen::Vector3d &pt3d){
    this->pt3d_ = pt3d;
}


void MapPoint::setPosition(const double x, const double y, const double z){
    this->pt3d_.x() = x;
    this->pt3d_.y() = y;
    this->pt3d_.z() = z;
}

void MapPoint::setColor(const Eigen::Vector3i &bgr){
    this->bgr_ = bgr;

}


void MapPoint::setColor(const int b, const int g, const int r){
    this->bgr_(0) = b;
    this->bgr_(1) = g;
    this->bgr_(2) = r;

}


int KeyPoint::id_counter_ = -1;



KeyPoint::KeyPoint():
id_(++KeyPoint::id_counter_),
timestamp_(-1),
map_point_ptr_(nullptr), 
next_keypoint_in_time_(nullptr), 
prev_keypoint_in_time_(nullptr),
left_keypoint_in_frame_(nullptr),
right_keypoint_in_frame_(nullptr)
{
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.queryIdx = -1;
    this->match_in_time_.imgIdx = id_;
    this->match_in_time_.distance = std::numeric_limits<float>::max();

    this->match_in_frame_.trainIdx = -1;
    this->match_in_frame_.queryIdx = -1;
    this->match_in_frame_.imgIdx = id_;
    this->match_in_frame_.distance = std::numeric_limits<float>::max();

}




KeyPoint::KeyPoint(const cv::KeyPoint kp):
KeyPoint()
{

    this->cv_keypoint_ = kp; 

}




void KeyPoint::setCVKeyPoint(const cv::KeyPoint &kp){
    this->cv_keypoint_ = kp;

}

void KeyPoint::setDescriptor(const cv::Mat &descriptor){
    this->descriptor_ = descriptor;
}

// void KeyPoint::setNextKeyPointInTime(const std::shared_ptr<KeyPoint> &next_keypoint_in_time){
//     this->next_keypoint_in_time_ = next_keypoint_in_time;
// }
// void KeyPoint::setPrevKeyPointInTime(const std::shared_ptr<KeyPoint> &prev_keypoint_in_time){
//     this->prev_keypoint_in_time_ = prev_keypoint_in_time;
// }

// void KeyPoint::setLeftKeyPointInFrame(const std::shared_ptr<KeyPoint> &left_keypoint_in_frame){
//     this->left_keypoint_in_frame_ = left_keypoint_in_frame;

// }
// void KeyPoint::setRightKeyPointInFrame(const std::shared_ptr<KeyPoint> &right_keypoint_in_frame){
//     this->right_keypoint_in_frame_ = right_keypoint_in_frame;

// }



// void KeyPoint::setMapPointPtr(const std::shared_ptr<MapPoint> &map_point_ptr){
//     this->map_point_ptr_ = map_point_ptr;
//     setMapPointPtrForward(map_point_ptr);
//     setMapPointPtrBackward(map_point_ptr);
//     setMapPointPtrLeftForward(map_point_ptr);
//     setMapPointPtrRightForward(map_point_ptr);

    
// }

// void KeyPoint::propagateMapPointPtr(){
//     if(this->map_point_ptr_ != nullptr){
//         setMapPointPtrForward(this->map_point_ptr_);
//         setMapPointPtrLeftForward(this->map_point_ptr_);
//         setMapPointPtrRightForward(this->map_point_ptr_);
//     }
//     else{
//         // LOG(INFO) << YELLOW << "trying propagate a null map point ptr";
//     }
// }




// void KeyPoint::setMapPointPtrForward(const std::shared_ptr<MapPoint> &map_point_ptr){
//     this->map_point_ptr_ = map_point_ptr;
//     if(this->next_keypoint_in_time_ != nullptr){
//         this->next_keypoint_in_time_->setMapPointPtrForward(map_point_ptr);
//     }
// }

// void KeyPoint::setMapPointPtrBackward(const std::shared_ptr<MapPoint> &map_point_ptr){
//     this->map_point_ptr_ = map_point_ptr;
//     if(this->prev_keypoint_in_time_ != nullptr){
//         this->prev_keypoint_in_time_->setMapPointPtrBackward(map_point_ptr);
//     }
// }

// void KeyPoint::setMapPointPtrLeftForward(const std::shared_ptr<MapPoint> &map_point_ptr){
//     this->map_point_ptr_ = map_point_ptr;
//     if(this->left_keypoint_in_frame_ != nullptr){
//         this->left_keypoint_in_frame_->setMapPointPtrLeftForward(map_point_ptr);
//     }
// }

// void KeyPoint::setMapPointPtrRightForward(const std::shared_ptr<MapPoint> &map_point_ptr){
//     this->map_point_ptr_ = map_point_ptr;
//     if(this->right_keypoint_in_frame_ != nullptr){
//         this->right_keypoint_in_frame_->setMapPointPtrRightForward(map_point_ptr);
//     }
// }



template <typename T>
void KeyPoint::set2DKeyPoint(const T x, const T y){

    this->cv_keypoint_ = cv::KeyPoint(cv::Point2f(x, y), 1.0f);

}


void KeyPoint::set2DKeyPoint(const cv::Point2f &pt2f){
    
    this->cv_keypoint_ = cv::KeyPoint(pt2f, 1.0f);

}

void KeyPoint::set3DKeyPoint(const double x, const double y, const double z){
    this->pt3d_.x = x;
    this->pt3d_.y = y;
    this->pt3d_.z = z;
}
void KeyPoint::set3DKeyPoint(const cv::Point3d &pt3d){
    this->pt3d_.x = pt3d.x;
    this->pt3d_.y = pt3d.y;
    this->pt3d_.z = pt3d.z;
}


void KeyPoint::setMatchInTime(const cv::DMatch &match_in_time){
    this->match_in_time_ = match_in_time;
}



void KeyPoint::setMatchInFrame(const cv::DMatch &match_in_frame){
    this->match_in_frame_ = match_in_frame;
}


void KeyPoint::cleanTrackInTimeRelationship(){
    this->match_in_time_.queryIdx = -1;
    this->match_in_time_.trainIdx = -1;
    this->match_in_time_.imgIdx   = -1;
    this->match_in_time_.distance = std::numeric_limits<float>::max();  // or 0.0f


}

void KeyPoint::cleanTrackInFrameRelationship(){

    this->match_in_frame_.queryIdx = -1;
    this->match_in_frame_.trainIdx = -1;
    this->match_in_frame_.imgIdx   = -1;
    this->match_in_frame_.distance = std::numeric_limits<float>::max();  // or 0.0f


}


} // namespace modules_vins





