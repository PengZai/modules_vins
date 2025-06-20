#include "descriptor_tracker.h"

namespace modules_vins{



DescriptorTracker::DescriptorTracker(const std::shared_ptr<SystemConfig> &sys_config):
Tracker(sys_config)
{

    this->bf_ = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);

}


void DescriptorTracker::matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, std::vector<cv::DMatch> &good_matches, const float error_threshold, const float y_distance_threshold){


    cv::Mat descriptors_from_img0, descriptors_from_img1;
    std::vector<cv::DMatch> matches;

    img0->getDescripots(descriptors_from_img0);
    img1->getDescripots(descriptors_from_img1);

    this->bf_->match(descriptors_from_img0, descriptors_from_img1, matches); // Find the two best matches

    // Sort matches based on distance (best matches first)
    float min_distance = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    int min_match_queryIdx = 1e20;
    int max_match_queryIdx = -1;
    int min_match_trainIdx = 1e20;
    int max_match_trainIdx = -1;

    for(size_t i=0; i< matches.size(); i++){

        cv::DMatch &match = matches.at(i);
        if(match.distance > std::max<double>(min_distance * this->sys_config_->params_->matching_ratio_, error_threshold)){
            // because mathces have been sorted, we no need to loop anymore once match distance larger than threshold
            continue;
        }

        if(y_distance_threshold != -1.0){
            double y_distance = std::abs(img0->keypoint_vector_[match.queryIdx]->cv_keypoint_.pt.y - img1->keypoint_vector_[match.trainIdx]->cv_keypoint_.pt.y);
            if(y_distance > y_distance_threshold){
                continue;
            }
        }
        

        match.imgIdx = img1->id_; // Store index of the train image
        good_matches.emplace_back(match);
        if(min_match_queryIdx > match.queryIdx) min_match_queryIdx = match.queryIdx;
        if(max_match_queryIdx < match.queryIdx) max_match_queryIdx = match.queryIdx;
        if(min_match_trainIdx > match.trainIdx) min_match_trainIdx = match.trainIdx;
        if(max_match_trainIdx < match.trainIdx) max_match_trainIdx = match.trainIdx;

   
    }

 
}




} //modules_vins

