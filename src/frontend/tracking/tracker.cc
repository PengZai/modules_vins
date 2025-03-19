#include "tracker.h"


namespace modules_vins{

  
Tracker::Tracker(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;
    this->bf_ = std::make_shared<BFMatcher>();
}

void Tracker::trackInFrame(Image &img0, Image &img1){

    std::vector<cv::DMatch> raw_matches;
    this->bf_->matching(img0, img1, raw_matches);
    for (int i=0; i < raw_matches.size(); i++) {
        cv::DMatch &match = raw_matches[i];
        if(match.distance < this->sys_config_->params_->threshold_for_tracking_descriptor_){
            continue;
        }
        // Manually assign `imgIdx`
        match.imgIdx = img1.sensor_id_; // Store index of the train image
        img0.matches_in_frame_.push_back(match);
    }
}

void Tracker::trackInTime(Image &img0, Image &img1){

    std::vector<cv::DMatch> raw_matches;
    this->bf_->matching(img0, img1, raw_matches);
    for (int i=0; i < raw_matches.size(); i++) {
        cv::DMatch &match = raw_matches[i];
        if(match.distance < this->sys_config_->params_->threshold_for_tracking_descriptor_){
            continue;
        }
        // Manually assign `imgIdx`
        match.imgIdx = img1.sensor_id_; // Store index of the train image
        img0.keypoint_vector_[match.queryIdx].setMatchInTime(match);
        img0.matches_in_time_.push_back(match);
    }
}

    
void Tracker::pipeline(CameraFrame &camera_frame){

    this->camera_frame_deque_.push_back(camera_frame);
    // track cross cameras (typically stereo trakcing)

    // we track feature according to the feature in camera 0(left camera)
    Image &img0 = camera_frame.image_vector_.at(0);
    for(int i=1;i<(int)camera_frame.image_vector_.size();i++){

        trackInFrame(img0, camera_frame.image_vector_.at(i));

    }

    CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();

    // we only track between current frame and previous frame
    if(this->camera_frame_deque_.size()>1){
        trackInTime(img0, previous_camera_frame.image_vector_.at(0));
        this->camera_frame_deque_.pop_front();
    }
    

}

    

    
    
} //modules_vins