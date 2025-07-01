#include "descriptor_tracker.h"

namespace modules_vins{



DescriptorTracker::DescriptorTracker(const std::shared_ptr<SystemConfig> &sys_config):
Tracker(sys_config)
{

    this->bf_ = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING, true);

}


void DescriptorTracker::matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, std::vector<cv::DMatch> &good_matches, const float error_threshold, const float y_distance_threshold){


    cv::Mat descriptors_from_img0, descriptors_from_img1;
    std::vector<cv::DMatch> matches;

    img0->getDescripots(descriptors_from_img0);
    img1->getDescripots(descriptors_from_img1);

    this->bf_->match(descriptors_from_img0, descriptors_from_img1, matches); // Find the two best matches

    float min_distance = 1e20;
    float max_distance = -1;

    for(size_t i=0; i< matches.size(); i++){
        cv::DMatch &match = matches.at(i);
        if(min_distance > match.distance){
            min_distance = match.distance;
        }
        if(max_distance < match.distance){
            max_distance = match.distance;
        }
    }

    // int min_match_queryIdx = 1e20;
    // int max_match_queryIdx = -1;
    // int min_match_trainIdx = 1e20;
    // int max_match_trainIdx = -1;

    for(size_t i=0; i< matches.size(); i++){

        cv::DMatch &match = matches.at(i);
        if(match.distance > std::max<double>(min_distance * this->sys_config_->params_->matching_ratio_, error_threshold)){
            // because mathces have been sorted, we no need to loop anymore once match distance larger than threshold
            continue;
        }

        // if(y_distance_threshold != -1.0){
        //     double y_distance = std::abs(img0->keypoint_vector_[match.queryIdx]->cv_keypoint_.pt.y - img1->keypoint_vector_[match.trainIdx]->cv_keypoint_.pt.y);
        //     if(y_distance > y_distance_threshold){
        //         continue;
        //     }
        // }
        

        match.imgIdx = img1->id_; // Store index of the train image
        good_matches.emplace_back(match);
        // if(min_match_queryIdx > match.queryIdx) min_match_queryIdx = match.queryIdx;
        // if(max_match_queryIdx < match.queryIdx) max_match_queryIdx = match.queryIdx;
        // if(min_match_trainIdx > match.trainIdx) min_match_trainIdx = match.trainIdx;
        // if(max_match_trainIdx < match.trainIdx) max_match_trainIdx = match.trainIdx;

   
    }

 
}

// trackInTime
void DescriptorTracker::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){


    std::shared_ptr<CameraFrame> &ref_camera_frame = camera_frame->ref_camera_frame_;

    if(ref_camera_frame == nullptr){

        LOG(INFO) << RED << "ref_camera_frame is nullptr" << RESET;
        return ;
    }

    std::shared_ptr<Image> &img0_from_current_frame = camera_frame->image_vector_.at(0);

    std::shared_ptr<Image> &img0_from_ref_frame = ref_camera_frame->image_vector_.at(0);
    
    ref_camera_frame->cleanTrackInTimeRelationship();
    camera_frame->cleanTrackInTimeRelationship();

    std::vector<cv::DMatch> matches;
    // we track feature according to the feature in camera 0(left camera)
    this->matching(img0_from_ref_frame, img0_from_current_frame, matches, this->sys_config_->feature_and_tracker_config_->orb_params_->threshold_for_tracking_descriptor_in_time_);
    ref_camera_frame->setTrackInTimeRelationship(matches);


    LOG(INFO) << GREEN << img0_from_ref_frame->matches_in_time_.size() << " points were trakced in time for camera frame bewteen ref " << ref_camera_frame->id_ << " and curr " << camera_frame->id_  << RESET;




}




void DescriptorTracker::trackInFrame(const std::shared_ptr<CameraFrame> &camera_frame){


    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);


    std::shared_ptr<Image> &img_1 = camera_frame->image_vector_.at(1);
    
    camera_frame->cleanTrackInFrameRelationship();
    std::vector<cv::DMatch> matches;
    this->matching(img_0, img_1, matches, 
        this->sys_config_->feature_and_tracker_config_->orb_params_->threshold_for_tracking_descriptor_in_frame_, 10);

    camera_frame->setTrackInFrameRelationship(matches);   

    LOG(INFO) << GREEN << img_0->matches_in_frame_.size() << " points were trakced in frame for camera frame " << camera_frame->id_  << RESET;

    
}



} //modules_vins

