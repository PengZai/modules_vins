#include "tracker.h"


namespace modules_vins{

  
Tracker::Tracker(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;
    this->bf_ = std::make_shared<BFMatcher>();

}

void Tracker::trackInFrame(CameraFrame &camera_frame){


    std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);
    for(int j=1;j<(int)camera_frame.image_vector_.size();j++){

        std::shared_ptr<Image> &img_j = camera_frame.image_vector_.at(j);
        std::vector<cv::DMatch> matches;
        this->bf_->matching(img_0, img_j, matches);
        for (int i=0; i < (int)matches.size(); i++) {
            cv::DMatch &match = matches[i];
            if(match.distance > this->sys_config_->params_->threshold_for_tracking_descriptor_in_frame_){
                // because mathces have been sorted, we no need to loop anymore once match distance larger than threshold
                break;
            }
            // Manually assign `imgIdx`
            match.imgIdx = img_j->sensor_id_; // Store index of the train image
            img_0->keypoint_vector_[match.queryIdx]->setMatchInFrame(match);
            img_0->matches_in_frame_.push_back(match);

            cv::DMatch match_for_img_j;
            match_for_img_j.queryIdx = match.trainIdx;  // Index of keypoint in the first image (query image)
            match_for_img_j.trainIdx = match.queryIdx;  // Index of keypoint in the second image (train image)
            match_for_img_j.imgIdx = 0;    // Index of the image in the train set (useful in multi-image matching)
            match_for_img_j.distance = match.distance; // Distance between the descriptors (lower is better)
            img_j->keypoint_vector_[match.trainIdx]->setMatchInFrame(match_for_img_j);
            img_j->matches_in_frame_.push_back(match_for_img_j);

        
        }
    }


    

    
}

void Tracker::trackInTime(CameraFrame &camera_frame){
    
    this->camera_frame_deque_.push_back(camera_frame);

    std::shared_ptr<Image> &img0_from_current_frame = camera_frame.image_vector_.at(0);

    // we only track between current frame and previous frame
    if(this->camera_frame_deque_.size()>1){

        CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();
        std::shared_ptr<Image> &img0_from_previous_frame = previous_camera_frame.image_vector_.at(0);

        std::vector<cv::DMatch> matches;
        this->bf_->matching(img0_from_current_frame, img0_from_previous_frame, matches);


        for (int i=0; i < (int)matches.size(); i++) {
            cv::DMatch &match = matches[i];
            if(match.distance > this->sys_config_->params_->threshold_for_tracking_descriptor_in_time_){
                // because mathces have been sorted, we no need to loop anymore once match distance larger than threshold
                break;
            }
            // Manually assign `imgIdx`
            match.imgIdx = img0_from_previous_frame->sensor_id_; // Store index of the train image

            std::shared_ptr<KeyPoint> &tracked_keypoint_from_current_frame = img0_from_current_frame->keypoint_vector_[match.queryIdx];
            std::shared_ptr<KeyPoint> &tracked_keypoint_from_previous_frame = img0_from_previous_frame->keypoint_vector_[match.trainIdx];

            tracked_keypoint_from_current_frame->setMatchInTime(match);
            img0_from_current_frame->matches_in_time_.push_back(match);

            tracked_keypoint_from_current_frame->setPrevKeyPointInTime(tracked_keypoint_from_previous_frame);
            tracked_keypoint_from_previous_frame->setNextKeyPointInTime(tracked_keypoint_from_current_frame);
            
        }   

        this->camera_frame_deque_.pop_front();
    }

    
}

    
void Tracker::pipeline(CameraFrame &camera_frame){


    // we track feature according to the feature in camera 0(left camera)
    trackInTime(camera_frame);

    // track cross cameras (typically stereo trakcing)
    trackInFrame(camera_frame);

    VLOG(VERBOSE) << GREEN << camera_frame.image_vector_.at(0)->matches_in_time_.size() << " points were trakced in time" << RESET;
    VLOG(VERBOSE) << GREEN << camera_frame.image_vector_.at(0)->matches_in_frame_.size() << " points were trakced in frame" << RESET; 


}

    

    
    
} //modules_vins