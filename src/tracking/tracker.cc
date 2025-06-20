#include "tracker.h"


namespace modules_vins{

  
Tracker::Tracker(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;

}


void Tracker::setDetector(const std::shared_ptr<Detector> &detector){
    this->detector_ = detector;
}

void Tracker::trackInFrame(const std::shared_ptr<CameraFrame> &camera_frame){


    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    for(int j=1;j<(int)camera_frame->image_vector_.size();j++){

        std::shared_ptr<Image> &img_j = camera_frame->image_vector_.at(j);
        
        std::vector<cv::DMatch> matches;
        this->matching(img_0, img_j, matches, this->sys_config_->params_->threshold_for_tracking_descriptor_in_frame_, 10);


        for (int i=0; i < (int)matches.size(); i++) {
            cv::DMatch &match = matches[i];
         


            std::shared_ptr<KeyPoint> &tracked_keypoint_from_img_0 = img_0->keypoint_vector_[match.queryIdx];
            std::shared_ptr<KeyPoint> &tracked_keypoint_from_img_j = img_j->keypoint_vector_[match.trainIdx];

            match.imgIdx = img_j->sensor_id_; // Store index of the train image
            if(tracked_keypoint_from_img_0 != nullptr){
                tracked_keypoint_from_img_0 ->setMatchInFrame(match);
            }

            img_0->matches_in_frame_.push_back(match);
            tracked_keypoint_from_img_0->setRightKeyPointInFrame(tracked_keypoint_from_img_j);


            // cv::DMatch match_for_img_j;
            // match_for_img_j.queryIdx = match.trainIdx;  // Index of keypoint in the first image (query image)
            // match_for_img_j.trainIdx = match.queryIdx;  // Index of keypoint in the second image (train image)
            // match_for_img_j.imgIdx = 0;    // Index of the image in the train set (useful in multi-image matching)
            // match_for_img_j.distance = match.distance; // Distance between the descriptors (lower is better)


            // std::shared_ptr<KeyPoint> &tracked_keypoint_from_img_j = img_j->keypoint_vector_[match.trainIdx];

            // if(tracked_keypoint_from_img_j != nullptr){
            //     tracked_keypoint_from_img_j->setMatchInFrame(match_for_img_j);
            // }
            // img_j->matches_in_frame_.push_back(match_for_img_j);

            // if(tracked_keypoint_from_img_0 != nullptr && tracked_keypoint_from_img_j != nullptr){
            //     tracked_keypoint_from_img_0->setRightKeyPointInFrame(tracked_keypoint_from_img_j);
            //     tracked_keypoint_from_img_j->setLeftKeyPointInFrame(tracked_keypoint_from_img_0);
            // }



            
        }
        LOG(INFO) << GREEN << img_0->matches_in_frame_.size() << " points were trakced in frame between " << img_0->sensor_id_  << " and " << img_j->sensor_id_ << RESET; 

    }


    

    
}


void Tracker::trackInTime(const std::shared_ptr<Image> &img_from_ref_frame, const std::shared_ptr<Image> &img_from_current_frame){
    

    std::vector<cv::DMatch> matches;
    this->matching(img_from_ref_frame, img_from_current_frame, matches, this->sys_config_->params_->threshold_for_tracking_descriptor_in_time_);
    // double min_distance = matches.front().distance;


    for (int i=0; i < (int)matches.size(); i++) {
        const cv::DMatch &match = matches[i];
  
        std::shared_ptr<KeyPoint> &tracked_keypoint_from_current_frame = img_from_current_frame->keypoint_vector_[match.trainIdx];
        std::shared_ptr<KeyPoint> &tracked_keypoint_from_ref_frame = img_from_ref_frame->keypoint_vector_[match.queryIdx];

        // tracked_keypoint_from_current_frame->setMatchInTime(match);
        // img_from_current_frame->matches_in_time_.push_back(match);
        // tracked_keypoint_from_current_frame->setPrevKeyPointInTime(tracked_keypoint_from_ref_frame);

        img_from_ref_frame->matches_in_time_.push_back(match);
        tracked_keypoint_from_ref_frame->setNextKeyPointInTime(tracked_keypoint_from_current_frame);
        tracked_keypoint_from_ref_frame->propagateMapPointPtr();
    }   



    LOG(INFO) << GREEN << img_from_ref_frame->matches_in_time_.size() << " points were trakced in time" << RESET;

    

    
}
    
void Tracker::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){


    std::shared_ptr<CameraFrame> &ref_camera_frame = camera_frame->ref_camera_frame_;

    if(ref_camera_frame == nullptr){

        LOG(INFO) << RED << "ref_camera_frame is nullptr" << RESET;
        return ;
    }

    std::shared_ptr<Image> &img0_from_current_frame = camera_frame->image_vector_.at(0);

    std::shared_ptr<Image> &img0_from_ref_frame = ref_camera_frame->image_vector_.at(0);
    

    // we track feature according to the feature in camera 0(left camera)
    trackInTime(img0_from_ref_frame, img0_from_current_frame);




}

    

    
    
} //modules_vins