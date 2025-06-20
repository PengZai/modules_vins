#include "KLT_tracker.h"



namespace modules_vins
{

KLTTracker::KLTTracker(const std::shared_ptr<SystemConfig> &sys_config):
Tracker(sys_config)
{

}


void KLTTracker::matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, std::vector<cv::DMatch> &good_matches, const float error_threshold, const float y_distance_threshold){


    std::vector<cv::Point2f> pt2fs_from_img0, pt2fs_from_img1;
    std::vector<uchar> status;
    std::vector<float> errors;

    int min_match_queryIdx = 1e20;
    int max_match_queryIdx = -1;
    int min_match_trainIdx = 1e20;
    int max_match_trainIdx = -1;

    for(size_t i=0; i<img0->keypoint_vector_.size();i++){
        std::shared_ptr<KeyPoint> &kp = img0->keypoint_vector_.at(i);
        pt2fs_from_img0.push_back(kp->cv_keypoint_.pt);
        pt2fs_from_img1.push_back(kp->cv_keypoint_.pt);
    }

    cv::calcOpticalFlowPyrLK(
        img0->gray_data_, img1->gray_data_,
        pt2fs_from_img0, pt2fs_from_img1, 
        status, errors,
    cv::Size(11, 11), 
    3,
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
    cv::OPTFLOW_USE_INITIAL_FLOW
    );

    int match_idx_for_img1 = 0;
    for(size_t i=0; i < status.size(); i++){

        
        if (!status[i]) {
            continue;

        }

        if(y_distance_threshold != -1){
            double y_distance = std::abs(pt2fs_from_img0.at(i).y - pt2fs_from_img1.at(i).y);
            if(y_distance > y_distance_threshold){
                continue;
            }

        }


        std::shared_ptr<KeyPoint> kp = std::make_shared<KeyPoint>(cv::KeyPoint(pt2fs_from_img1[i], 3));
        img1->keypoint_vector_.emplace_back(kp);
        cv::DMatch match;
        match.queryIdx = i;
        match.trainIdx = match_idx_for_img1;
        match.imgIdx = img1->id_;
        match.distance = errors[i];
        good_matches.emplace_back(match);

        match_idx_for_img1++;
    }


    for(size_t i=0;i<good_matches.size();i++){
        
        cv::DMatch &match = good_matches.at(i);

        if(min_match_queryIdx > match.queryIdx) min_match_queryIdx = match.queryIdx;
        if(max_match_queryIdx < match.queryIdx) max_match_queryIdx = match.queryIdx;
        if(min_match_trainIdx > match.trainIdx) min_match_trainIdx = match.trainIdx;
        if(max_match_trainIdx < match.trainIdx) max_match_trainIdx = match.trainIdx;

    }
    

}





} // namespace modules_vins
