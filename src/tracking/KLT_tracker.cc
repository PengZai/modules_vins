#include "KLT_tracker.h"



namespace modules_vins
{

KLTTracker::KLTTracker(const std::shared_ptr<SystemConfig> &sys_config):
Tracker(sys_config)
{


}


void KLTTracker::matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, std::vector<cv::DMatch> &good_matches, const int max_count, const float epsilon, const float y_distance_threshold){


    std::vector<cv::Point2f> pt2fs_from_img0, pt2fs_from_img1;
    std::vector<uchar> status;
    std::vector<float> errors;

    int min_match_queryIdx = 1e20;
    int max_match_queryIdx = -1;
    int min_match_trainIdx = 1e20;
    int max_match_trainIdx = -1;

    Eigen::Matrix<double, 3, 3> K_j = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img1->sensor_id_)->getIntrinsicsMatrix();         
    
    Eigen::Matrix<double, 4, 4> Tc1c0= this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(img1->sensor_id_, img0->sensor_id_);
    Eigen::Matrix<double, 4,4 > Tc0b = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(0)->T_base_sensor_.inverse().matrix();



    for(size_t i=0; i<img0->keypoint_vector_.size();i++){
        std::shared_ptr<KeyPoint> &kp = img0->keypoint_vector_.at(i);
        cv::Point2f &pt2f_from_img0 = kp->cv_keypoint_.pt;
        pt2fs_from_img0.push_back(pt2f_from_img0);

        if(kp->map_point_ptr_){
            std::shared_ptr<MapPoint> mp = kp->map_point_ptr_;

            Eigen::Matrix<double, 4,4> T_c1_w = Tc1c0 * Tc0b * img0->frame_->T_b_w_.matrix() ;
            Eigen::Vector3d pj =  T_c1_w.block<3,4>(0,0) * mp->pt3d_.homogeneous();
            Eigen::Vector2d reprojected_pt = camera2pixel(pj, K_j);
            pt2fs_from_img1.push_back(cv::Point2f(reprojected_pt(0), reprojected_pt(1)));

        }
        else{
            pt2fs_from_img1.push_back(kp->cv_keypoint_.pt);
        }
    }

    cv::calcOpticalFlowPyrLK(
        img0->gray_data_, img1->gray_data_,
        pt2fs_from_img0, pt2fs_from_img1, 
        status, errors,
    cv::Size(11, 11), 
    3,
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
    max_count,
    0.01),
    cv::OPTFLOW_USE_INITIAL_FLOW
    );

    int match_idx_for_img1 = 0;
    std::vector<size_t> lost_track_ids;
    for(size_t i=0; i < status.size(); i++){

        
        if (!status[i]) {
            lost_track_ids.push_back(i);
            continue;

        }

        // if(y_distance_threshold != -1){
        //     double y_distance = std::abs(pt2fs_from_img0.at(i).y - pt2fs_from_img1.at(i).y);
        //     if(y_distance > y_distance_threshold){
        //         continue;
        //     }
        // }

        // if(img1->isInImage(pt2fs_from_img1[i]) == false){
        //     continue;
        // }


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


    // for(size_t i=0;i<good_matches.size();i++){
        
    //     cv::DMatch &match = good_matches.at(i);

    //     if(min_match_queryIdx > match.queryIdx) min_match_queryIdx = match.queryIdx;
    //     if(max_match_queryIdx < match.queryIdx) max_match_queryIdx = match.queryIdx;
    //     if(min_match_trainIdx > match.trainIdx) min_match_trainIdx = match.trainIdx;
    //     if(max_match_trainIdx < match.trainIdx) max_match_trainIdx = match.trainIdx;

    // }

    LOG(INFO) << "Just test";
    

}


// trackInTime
void KLTTracker::pipeline(const std::shared_ptr<Frame> &frame){


    std::shared_ptr<Frame> &ref_frame = frame->ref_frame_;

    if(ref_frame == nullptr){

        LOG(INFO) << RED << "ref_frame is nullptr" << RESET;
        return ;
    }

    ref_frame->cleanTrackInTimeRelationship();
    frame->cleanFeaturePoints();
    frame->cleanTrackInTimeRelationship();

    std::shared_ptr<Image> &img0_from_current_frame = frame->image_vector_.at(0);

    std::shared_ptr<Image> &img0_from_ref_frame = ref_frame->image_vector_.at(0);
    

    // we track feature according to the feature in camera 0(left camera)
    std::vector<cv::DMatch> matches;
    this->matching(img0_from_ref_frame, img0_from_current_frame, matches, 
        this->sys_config_->feature_and_tracker_config_->klt_params_->max_count_, 
        this->sys_config_->feature_and_tracker_config_->klt_params_->epsilon_);

    cv::Mat cv_K = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img0_from_current_frame->sensor_id_)->getCVIntrinsicsMatrix();         
    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img0_from_current_frame->sensor_id_)->getCVDistortionCoeffs();


    for(int i=0; i < matches.size(); i++){
        const cv::DMatch &match = matches.at(i);
        const std::shared_ptr<KeyPoint> &kp = img0_from_current_frame->keypoint_vector_.at(match.trainIdx);
        std::vector<cv::Point2f> pt_distorted = {kp->cv_keypoint_.pt};
        std::vector<cv::Point2f> pt_undistorted;
        cv::undistortPoints(pt_distorted, pt_undistorted, cv_K, cv_distortion_coeffs);
        kp->undistorted_pt2d_ = pt_undistorted[0];
    }

    ref_frame->setTrackInTimeRelationship(matches);

    LOG(INFO) << GREEN << img0_from_ref_frame->matches_in_time_.size() << " points were trakced in time for camera frame bewteen ref " << ref_frame->id_ << " and curr " << frame->id_  << RESET;


}





void KLTTracker::trackInFrame(const std::shared_ptr<Frame> &frame){


    std::shared_ptr<Image> &img_0 = frame->image_vector_.at(0);


    std::shared_ptr<Image> &img_1 = frame->image_vector_.at(1);
    
        
    frame->cleanTrackInFrameRelationship();

    std::vector<cv::DMatch> matches;
    this->matching(img_0, img_1, matches, 
        this->sys_config_->feature_and_tracker_config_->klt_params_->max_count_,
        this->sys_config_->feature_and_tracker_config_->klt_params_->epsilon_,
        10);


    cv::Mat cv_K_from_img_0 = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_0->sensor_id_)->getCVIntrinsicsMatrix();         
    cv::Mat cv_distortion_coeffs_from_img_0 = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_0->sensor_id_)->getCVDistortionCoeffs();
    cv::Mat cv_K_from_img_1 = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_1->sensor_id_)->getCVIntrinsicsMatrix();         
    cv::Mat cv_distortion_coeffs_from_img_1 = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_1->sensor_id_)->getCVDistortionCoeffs();


    for(int i=0; i < matches.size(); i++){
        const cv::DMatch &match = matches.at(i);
        const std::shared_ptr<KeyPoint> &kp_from_img_0 = img_0->keypoint_vector_.at(match.queryIdx);
        const std::shared_ptr<KeyPoint> &kp_from_img_1 = img_1->keypoint_vector_.at(match.trainIdx);
        std::vector<cv::Point2f> pt_distorted_from_img_0 = {kp_from_img_0->cv_keypoint_.pt};
        std::vector<cv::Point2f> pt_distorted_from_img_1 = {kp_from_img_1->cv_keypoint_.pt};
        std::vector<cv::Point2f> pt_undistorted_from_img_0, pt_undistorted_from_img_1;
        cv::undistortPoints(pt_distorted_from_img_0, pt_undistorted_from_img_0, cv_K_from_img_0, cv_distortion_coeffs_from_img_0);
        cv::undistortPoints(pt_distorted_from_img_1, pt_undistorted_from_img_1, cv_K_from_img_1, cv_distortion_coeffs_from_img_1);

        kp_from_img_0->undistorted_pt2d_ = pt_undistorted_from_img_0[0];
        kp_from_img_1->undistorted_pt2d_ = pt_undistorted_from_img_1[0];

    }

    frame->setTrackInFrameRelationship(matches);   


    LOG(INFO) << GREEN << img_0->matches_in_frame_.size() << " points were trakced in frame for camera frame " << frame->id_  << RESET;

    
}



} // namespace modules_vins
