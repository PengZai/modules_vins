#include "pose_estimator.h"



namespace modules_vins
{
    

PoseEstimator::PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{


}


bool PoseEstimator::checkEstimatedPose(const Sophus::SE3<double> &estimated_T_b_w, const Sophus::SE3<double> &initial_guess_T_b_w, const int num_inliers){



    const Sophus::SE3<double> relative_T = estimated_T_b_w * initial_guess_T_b_w.inverse();
    Sophus::Vector6d relative_d = relative_T.log();
    double relative_d_norm = relative_d.norm();

    LOG(INFO) << "number of inliers: " << num_inliers;
    LOG(INFO) << "estimated_Transformation: \n" << estimated_T_b_w.matrix();
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
    Sophus::SE3<double> &estimated_T_b_w,
    const cv::Mat K,
    const cv::Mat cv_distortion_coeffs
){

    Sophus::SE3<double> initial_guess_T_b_w = estimated_T_b_w;

    cv::Mat cv_R, rortation_vec, translation_vec;
    cv::eigen2cv(estimated_T_b_w.rotationMatrix(), cv_R);
    cv::eigen2cv(estimated_T_b_w.translation(), translation_vec);
    cv::Rodrigues(cv_R, rortation_vec);

    cv::Mat inliers;

    bool success = cv::solvePnPRansac(
    pt3ds,                 // std::vector<cv::Point3d>
    pt2ds,                  // std::vector<cv::Point2d>
    K,                       // Intrinsic matrix
    cv_distortion_coeffs,       // Distortion coefficients
    rortation_vec,                          // Output: rotation vector
    translation_vec,                          // Output: translation vector
    true,                         // Use extrinsic guess? Usually false
    100,                           // RANSAC iterations
    4.0,                           // Reprojection error threshold (pixels)
    0.99,                          // Confidence
    inliers                       // Output: inlier indices
    );

    // bool success = cv::solvePnP(
    //     pt3ds,                 // std::vector<cv::Point3d>
    //     pt2ds,                  // std::vector<cv::Point2d>
    //     K,                       // Intrinsic matrix
    //     cv_distortion_coeffs,       // Distortion coefficients
    //     rortation_vec,                          // Output: rotation vector
    //     translation_vec,                          // Output: translation vector
    //     true                         // Use extrinsic guess? Usually false
    // );

    cv::Rodrigues(rortation_vec, cv_R);


    Eigen::Matrix<double, 3, 3> estimated_rotation;
    Eigen::Vector3d estimated_translation;

    cv::cv2eigen(cv_R, estimated_rotation);
    cv::cv2eigen(translation_vec, estimated_translation);

    estimated_T_b_w = Sophus::SE3<double>(
        Sophus::SO3<double>(estimated_rotation), estimated_translation
    );

    LOG(INFO) << "Pnp in image plane:\n" << estimated_T_b_w.matrix();

    return inliers.rows;


}


int PoseEstimator::PnpEstimator(
    const std::vector<cv::Point3d> &pt3ds, 
    const std::vector<cv::Point2d> &pt2ds, 
    Sophus::SE3<double> &estimated_T_b_w
){

    Sophus::SE3<double> initial_guess_T_b_w = estimated_T_b_w;

    cv::Mat cv_R, rortation_vec, translation_vec;
    cv::eigen2cv(estimated_T_b_w.rotationMatrix(), cv_R);
    cv::eigen2cv(estimated_T_b_w.translation(), translation_vec);
    cv::Rodrigues(cv_R, rortation_vec);

    cv::Mat inliers;

    // bool success = cv::solvePnPRansac(
    // pt3ds,                 // std::vector<cv::Point3d>
    // pt2ds,                  // std::vector<cv::Point2d>
    // cv::Mat::eye(3,3,CV_64F),                       // Intrinsic matrix
    // cv::Mat(),       // Distortion coefficients
    // rortation_vec,                          // Output: rotation vector
    // translation_vec,                          // Output: translation vector
    // true,                         // Use extrinsic guess? Usually false
    // 100,                           // RANSAC iterations
    // 4.0,                           // Reprojection error threshold (pixels)
    // 0.99,                          // Confidence
    // inliers                       // Output: inlier indices
    // );

    bool success = cv::solvePnP(
        pt3ds,                 // std::vector<cv::Point3d>
        pt2ds,                  // std::vector<cv::Point2d>
        cv::Mat::eye(3,3,CV_64F),                       // Intrinsic matrix
        cv::Mat(),       // Distortion coefficients
        rortation_vec,                          // Output: rotation vector
        translation_vec,                          // Output: translation vector
        true                         // Use extrinsic guess? Usually false
    );

    cv::Rodrigues(rortation_vec, cv_R);


    Eigen::Matrix<double, 3, 3> estimated_rotation;
    Eigen::Vector3d estimated_translation;

    cv::cv2eigen(cv_R, estimated_rotation);
    cv::cv2eigen(translation_vec, estimated_translation);

    estimated_T_b_w = Sophus::SE3<double>(
        Sophus::SO3<double>(estimated_rotation), estimated_translation
    );

    LOG(INFO) << "Pnp in normalized plane:\n" << estimated_T_b_w.matrix();

    return 100; 

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



bool PoseEstimator::pipeline(const std::shared_ptr<Frame> &frame, int &num_inlier, double maximum_motion_norm){

    std::shared_ptr<Frame> &ref_frame = frame->ref_frame_;
    if(ref_frame == nullptr){

        LOG(INFO) << RED << "ref_frame is nullptr" << RESET;
        return -1;
    }

    num_inlier = 0;

    std::shared_ptr<Image> &img_0_from_current_frame = frame->image_vector_.at(0);

    // we only estimate pose between current frame and previous frame
    std::shared_ptr<Image> &img_0_from_ref_frame = ref_frame->image_vector_.at(0);

    std::vector<cv::Point2d> pt2ds, pt2ds_;
    std::vector<cv::Point2d> reprojected_pt2ds;
    std::vector<cv::Point2d> prev_pt2ds;
    std::vector<cv::Point3d> pt3ds, pt3ds_;
    std::vector<int> valid_idxes;

    std::vector<Eigen::Vector2d> eigen_pt2ds;
    std::vector<Eigen::Vector3d> eigen_pt3ds;

    const Eigen::Matrix3d K = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(0)->getIntrinsicsMatrix();
    cv::Mat cv_K = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_0_from_current_frame->sensor_id_)->getCVIntrinsicsMatrix();         

    const Eigen::VectorXd distortion_coeffs = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_0_from_current_frame->sensor_id_)->getDistortionCoeffs();
    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_0_from_current_frame->sensor_id_)->getCVDistortionCoeffs();


    for(size_t i=0; i < (int)img_0_from_ref_frame->matches_in_time_.size();i++){

        cv::DMatch &match_in_time = img_0_from_ref_frame->matches_in_time_.at(i);
        const std::shared_ptr<KeyPoint> &kp_from_ref_frame = img_0_from_ref_frame->keypoint_vector_.at(match_in_time.queryIdx);
        const std::shared_ptr<KeyPoint> &kp_from_current_frame = img_0_from_current_frame->keypoint_vector_.at(match_in_time.trainIdx);

        
        if(kp_from_ref_frame->map_point_ptr_ == nullptr){
            continue;
        }

        Eigen::Vector3d &w_pt3d = kp_from_ref_frame->map_point_ptr_->pt3d_;
        double z = w_pt3d(2);
        // Eigen::Vector3d &w_pt3d_ = kp_from_ref_frame->map_point_ptr2_->pt3d_;
        // double z2 = w_pt3d_(2);

        // if(z <= 0 || z2 <= 0){
        //     continue;
        // }
        if(z <= 0){
            continue;
        }

        // Eigen::Vector2d eigen_pt2d = Eigen::Vector2d(kp_from_current_frame->undistorted_pt2d_.x, kp_from_current_frame->undistorted_pt2d_.y);
        Eigen::Vector2d eigen_pt2d = Eigen::Vector2d(kp_from_current_frame->cv_keypoint_.pt.x, kp_from_current_frame->cv_keypoint_.pt.y);
        cv::Point3d w_cv_pt3d = cv::Point3d(w_pt3d(0), w_pt3d(1), w_pt3d(2));
        // cv::Point3d w_cv_pt3d_ = cv::Point3d(w_pt3d_(0), w_pt3d_(1), w_pt3d_(2));

        valid_idxes.push_back(match_in_time.queryIdx);
        pt2ds.push_back(kp_from_current_frame->cv_keypoint_.pt);
        // pt2ds.push_back(cv::Point2d(kp_from_current_frame->undistorted_pt2d_.x, kp_from_current_frame->undistorted_pt2d_.y));
        cv::Point2d normalized_pt2d = pixel2norm(kp_from_current_frame->cv_keypoint_.pt, cv_K);
        // reprojected_pt2ds.push_back(camera2pixel(w_cv_pt3d, cv_K));
        // prev_pt2ds.push_back(kp_from_ref_frame->cv_keypoint_.pt);
        pt3ds.push_back(w_cv_pt3d);
        // pt3ds_.push_back(w_cv_pt3d_);
        eigen_pt2ds.push_back(eigen_pt2d);
        eigen_pt3ds.push_back(w_pt3d);

    }

    // for(int j=0;j<pt3ds.size();j++){
    //     LOG(INFO) << "prev:" << prev_pt2is.at(j).x << " projected:"<< reprojected_pt2is.at(j).x << ":" << "curr" << pt2is.at(j).x;
    // }

    // checkImages(img_0_from_previous_frame, img_0_from_current_frame);

    int pt3ds_size = eigen_pt3ds.size();
    if(pt3ds_size < this->sys_config_->params_->min_inliers_){
        frame->status_ = Frame::FAIL;
        LOG(INFO) << "number of 3d points " << pt3ds_size << " , is less then : " << this->sys_config_->params_->min_inliers_ << " in pnp estimation";

        return false;
    }

    Sophus::SE3<double> estimated_T_b_w = frame->T_b_w_;

    // Sophus::SE3<double> estimated_T_c_w_ = Sophus::SE3<double>();
    // Sophus::SE3<double> estimated_T_c_w = Sophus::SE3<double>();

    bool success = false;

    // LOG(INFO) << "K" << cv_K;
    // LOG(INFO) << "cv_distortion_coeffs" << cv_distortion_coeffs;

    // num_inlier = PnpEstimator(pt3ds, pt2ds, estimated_T_c_w, cv_K, cv_distortion_coeffs);

    // if(checkEstimatedPose(estimated_T_c_w, img_0_from_ref_frame->T_c_w_, num_inlier) == true){
    //     success = true;
    // }

    // if(success == false){
        
    //     frame->status_=Frame::Status::FAIL;
    //     return success;
    // }

    LOG(INFO) << "before_BA\n" << frame->T_b_w_.matrix();


    num_inlier = bundleAdjustmentPoseOnlyCeres(eigen_pt3ds, eigen_pt2ds, K, estimated_T_b_w);
    // success = bundleAdjustmentPoseOnlyCeres(eigen_pt3ds, eigen_pt2ds, estimated_T_c_w);

    const Sophus::SE3d T_curr_ref = estimated_T_b_w * ref_frame->T_b_w_.inverse();

    double norm_T_curr_ref = T_curr_ref.log().norm();

    const double dt = img_0_from_current_frame->timestamp_ - img_0_from_ref_frame->timestamp_;

    const Sophus::Vector6d Velocity_T_b_w = T_curr_ref.log()/dt;

    if(norm_T_curr_ref < maximum_motion_norm && num_inlier > this->sys_config_->params_->min_inliers_){
        LOG(INFO) << GREEN << "norm_T_curr_ref :" << norm_T_curr_ref << RESET;
        success = true;
    }

    if(success == false){
        
        LOG(INFO) << YELLOW << " too large motion : " << norm_T_curr_ref << " or too few inliner :" << num_inlier << RESET;
        frame->status_=Frame::Status::FAIL;
        return num_inlier;
    }

    
    // update 3d points after BA
    // for(size_t i=0 ; i<eigen_pt3ds.size();i++){

    //     const std::shared_ptr<MapPoint> & map_point_ptr = img_0_from_ref_frame->keypoint_vector_.at(valid_idxes[i])->map_point_ptr_;
    //     map_point_ptr->setPosition(eigen_pt3ds.at(i));
    // }

    // LOG(INFO) << "after BA\n" << estimated_T_c_w.matrix();


    frame->setTbw(estimated_T_b_w);
    // const double dt = 1;
    frame->setVelocity(Velocity_T_b_w);

    // double relative_T_curr_ref_norm = this->relative_T_curr_ref.log().norm();
    // LOG(INFO) << "relative_T_curr_ref_norm : " << relative_T_curr_ref_norm;
    frame->status_=Frame::Status::NORMAL;
    // frame->status_=Frame::Status::FAIL;

    return success;


}

} // namespace modules_vins

