#include "tracker.h"


namespace modules_vins{

  
Tracker::Tracker(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;
    this->bf_ = std::make_shared<BFMatcher>();
}

void Tracker::track(Image &img0, Image &img1){


    this->bf_->matching(img0.descriptors_, img0.descriptors_);
}

    
void Tracker::pipeline(CameraFrame &camera_frame){

    this->camera_frame_deque_.push_back(camera_frame);
    // track cross cameras (typically stereo trakcing)

    // we track feature according to the feature in camera 0(left camera)
    Image &img0 = camera_frame.image_vector_.at(0);
    for(int i=1;i<(int)camera_frame.image_vector_.size();i++){

        track(img0, camera_frame.image_vector_.at(i));

    }

    CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();

    // we only track between current frame and previous frame
    if(this->camera_frame_deque_.size()>1){
        track(img0, previous_camera_frame.image_vector_.at(0));
        this->camera_frame_deque_.pop_front();
    }
    

}

    

    
    
} //modules_vins