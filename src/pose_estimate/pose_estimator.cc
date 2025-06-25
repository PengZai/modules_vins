#include "pose_estimator.h"



namespace modules_vins
{
    

PoseEstimator::PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

    this->relative_T_curr_ref =  Sophus::SE3<double>();

}


bool PoseEstimator::checkEstimatedPose(const Sophus::SE3<double> &estimated_T,const Sophus::SE3<double> &initial_guess, const int num_inliers){



    const Sophus::SE3<double> relative_T = estimated_T * initial_guess.inverse();
    Sophus::Vector6d relative_d = relative_T.log();
    double relative_d_norm = relative_d.norm();

    LOG(INFO) << "number of inliers: " << num_inliers;
    LOG(INFO) << "estimated_Transformation: \n" << estimated_T.matrix();
    LOG(INFO) << "the norm of estimated_Transformation norm: " << relative_d_norm;
    
    

    if(num_inliers < this->sys_config_->params_->min_inliers_)
    {
        LOG(INFO) << "reject because inlier is too small: " << num_inliers;
        return false;
    }

    if(relative_d_norm > this->sys_config_->params_->threshold_for_pnp_pose_log_norm_){

        LOG(INFO) <<"reject because motion is too large: " << relative_d_norm;
        return false;
    }

    return true;
    
}

int PoseEstimator::PnpEstimator(
    const std::vector<cv::Point3d> &pt3ds, 
    const std::vector<cv::Point2d> &pt2ds, 
    Sophus::SE3<double> &estimated_T,
    const cv::Mat &cv_K,
    const cv::Mat &cv_distortion_coeffs
){

    Sophus::SE3<double> initial_guess_T = estimated_T;

    cv::Mat cv_R, rortation_vec, translation_vec;
    cv::eigen2cv(estimated_T.rotationMatrix(), cv_R);
    cv::eigen2cv(estimated_T.translation(), translation_vec);
    cv::Rodrigues(cv_R, rortation_vec);

    cv::Mat inliers;

    bool success = cv::solvePnPRansac(
    pt3ds,                 // std::vector<cv::Point3d>
    pt2ds,                  // std::vector<cv::Point2d>
    cv_K,                       // Intrinsic matrix
    cv_distortion_coeffs,       // Distortion coefficients
    rortation_vec,                          // Output: rotation vector
    translation_vec,                          // Output: translation vector
    true,                         // Use extrinsic guess? Usually false
    100,                           // RANSAC iterations
    4.0,                           // Reprojection error threshold (pixels)
    0.99,                          // Confidence
    inliers                       // Output: inlier indices
    );

    cv::Rodrigues(rortation_vec, cv_R);


    Eigen::Matrix<double, 3, 3> estimated_rotation;
    Eigen::Vector3d estimated_translation;

    cv::cv2eigen(cv_R, estimated_rotation);
    cv::cv2eigen(translation_vec, estimated_translation);

    estimated_T = Sophus::SE3<double>(
        Sophus::SO3<double>(estimated_rotation), estimated_translation
    );

    // LOG(INFO) << "Pnp:\n" << estimated_T.matrix();

    return inliers.rows;


}

void checkImages(const std::shared_ptr<Image> &prev_img, const std::shared_ptr<Image> &img){

    std::string prev_img_name = std::to_string(prev_img->timestamp_) + ".png";
    std::string img_name = std::to_string(img->timestamp_) + ".png";

    cv::imshow(prev_img_name, prev_img->color_data_);
    cv::imshow(img_name, img->color_data_);

    cv::waitKey(0);
}


int PoseEstimator::epipolarGeometryEstimator(
    const std::vector<cv::Point2d> &ref_pt2ds, 
    const std::vector<cv::Point2d> &pt2ds, 
    cv::Mat &cv_R, 
    cv::Mat &translation_vec,
    const cv::Mat &cv_K
){

    cv::Mat inlierMask;
    cv::Mat cv_E = cv::findEssentialMat(pt2ds, ref_pt2ds, cv_K, cv::RANSAC, 0.999, 1.0, inlierMask);

    int inlier_num = cv::recoverPose(cv_E, pt2ds, ref_pt2ds,  cv_K, cv_R, translation_vec, inlierMask);

    return inlier_num;

}



void PoseEstimator::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){

    std::shared_ptr<CameraFrame> &ref_camera_frame = camera_frame->ref_camera_frame_;
    if(ref_camera_frame == nullptr){

        LOG(INFO) << RED << "ref_camera_frame is nullptr" << RESET;
        return ;
    }


    std::shared_ptr<Image> &img_0_from_current_frame = camera_frame->image_vector_.at(0);

    // we only estimate pose between current frame and previous frame
    std::shared_ptr<Image> &img_0_from_ref_frame = ref_camera_frame->image_vector_.at(0);

    std::vector<cv::Point2d> pt2ds;
    std::vector<cv::Point2d> reprojected_pt2ds;
    std::vector<cv::Point2d> prev_pt2ds;
    std::vector<cv::Point3d> pt3ds;
    std::vector<int> valid_idxes;

    std::vector<Eigen::Vector2d> eigen_pt2ds;
    std::vector<Eigen::Vector3d> eigen_pt3ds;

    const Eigen::Matrix3d K = this->sys_config_->camera_config_->params_vector_.at(0)->getIntrinsicsMatrix();
    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(0)->getCVIntrinsicsMatrix();         

    const Eigen::VectorXd distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(0)->getDistortionCoeffs();
    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(0)->getCVDistortionCoeffs();


    for(size_t i=0; i < (int)img_0_from_ref_frame->matches_in_time_.size();i++){

        cv::DMatch &match_in_time = img_0_from_ref_frame->matches_in_time_.at(i);
        const std::shared_ptr<KeyPoint> &kp_from_ref_frame = img_0_from_ref_frame->keypoint_vector_.at(match_in_time.queryIdx);
        const std::shared_ptr<KeyPoint> &kp_from_current_frame = img_0_from_current_frame->keypoint_vector_.at(match_in_time.trainIdx);

        
        if(kp_from_ref_frame->map_point_ptr_ == nullptr){
            continue;
        }

        Eigen::Vector3d &w_pt3d = kp_from_ref_frame->map_point_ptr_->pt3d_;
        double z = w_pt3d(2);

        if(z <= 0){
            continue;
        }

        Eigen::Vector2d eigen_pt2d = Eigen::Vector2d(kp_from_current_frame->cv_keypoint_.pt.x, kp_from_current_frame->cv_keypoint_.pt.y);
        cv::Point3d w_cv_pt3d = cv::Point3d(w_pt3d(0), w_pt3d(1), w_pt3d(2));

        valid_idxes.push_back(match_in_time.queryIdx);
        pt2ds.push_back(kp_from_current_frame->cv_keypoint_.pt);
        reprojected_pt2ds.push_back(camera2pixel(w_cv_pt3d, cv_K));
        prev_pt2ds.push_back(kp_from_ref_frame->cv_keypoint_.pt);
        pt3ds.push_back(w_cv_pt3d);
        eigen_pt2ds.push_back(eigen_pt2d);
        eigen_pt3ds.push_back(w_pt3d);

    }

    // for(int j=0;j<pt3ds.size();j++){
    //     LOG(INFO) << "prev:" << prev_pt2is.at(j).x << " projected:"<< reprojected_pt2is.at(j).x << ":" << "curr" << pt2is.at(j).x;
    // }

    // checkImages(img_0_from_previous_frame, img_0_from_current_frame);

    int pt3ds_size = pt3ds.size();
    if(pt3ds_size < this->sys_config_->params_->min_inliers_){
        camera_frame->status_ = CameraFrame::FAIL;
        LOG(INFO) << "number of 3d points " << pt3ds_size << " , is less then : " << this->sys_config_->params_->min_inliers_ << " in pnp estimation";

        return;
    }


    Sophus::SE3<double> estimated_T_c_w = this->relative_T_curr_ref * img_0_from_ref_frame->T_c_w_;
    bool success=false;
    // int inlier_rows = PnpEstimator(pt3ds, pt2ds, estimated_T_c_w, cv_K, cv_distortion_coeffs);

    // if(checkEstimatedPose(estimated_T_c_w, img_0_from_ref_frame->T_c_w_, inlier_rows) == true){
    //     success = true;
    // }

    // if(success == false){
        
    //     camera_frame->status_=CameraFrame::Status::FAIL;
    //     return;
    // }

    // LOG(INFO) << "before_BA\n" << img_0_from_current_frame->T_c_w_.matrix();


    success = bundleAdjustmentPoseOnlyCeres(eigen_pt3ds, eigen_pt2ds, K, estimated_T_c_w);
    if(success == false){
        
        camera_frame->status_=CameraFrame::Status::FAIL;
        return;
    }
    
    // update 3d points after BA
    // for(size_t i=0 ; i<eigen_pt3ds.size();i++){

    //     const std::shared_ptr<MapPoint> & map_point_ptr = img_0_from_ref_frame->keypoint_vector_.at(valid_idxes[i])->map_point_ptr_;
    //     map_point_ptr->setPosition(eigen_pt3ds.at(i));
    // }

    // LOG(INFO) << "after BA\n" << estimated_T_c_w.matrix();


    img_0_from_current_frame->setTcw(estimated_T_c_w);
    this->relative_T_curr_ref = img_0_from_ref_frame->T_c_w_.inverse() * estimated_T_c_w;
    // double relative_T_curr_ref_norm = this->relative_T_curr_ref.log().norm();
    // LOG(INFO) << "relative_T_curr_ref_norm : " << relative_T_curr_ref_norm;
    camera_frame->status_=CameraFrame::Status::NORMAL;
    // camera_frame->status_=CameraFrame::Status::FAIL;


}

} // namespace modules_vins

