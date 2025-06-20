#include "pose_estimator.h"



namespace modules_vins
{
    

PoseEstimator::PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

}


bool PoseEstimator::checkEstimatedPose(const Sophus::SE3<double> &Transformation,const int num_inliers){

    Sophus::Vector6d d = Transformation.log();
    double d_norm = d.norm();

    LOG(INFO) << "number of inliers: " << num_inliers;
    LOG(INFO) << "estimated_Transformation: \n" << Transformation.matrix();
    LOG(INFO) << "the norm of estimated_Transformation norm: " << d_norm;
    
    

    if(num_inliers < this->sys_config_->params_->min_inliers_)
    {
        LOG(INFO) << "reject because inlier is too small: " << num_inliers;
        return false;
    }

    if(d_norm > this->sys_config_->params_->threshold_for_pnp_pose_log_norm_){

        LOG(INFO) <<"reject because motion is too large: " << d_norm;
        return false;
    }

    return true;
    
}

bool PoseEstimator::PnpEstimator(
    const std::vector<cv::Point3d> &pt3ds, 
    const std::vector<cv::Point2d> &pt2ds, 
    Sophus::SE3<double> &estimated_T,
    const cv::Mat &cv_K,
    const cv::Mat &cv_distortion_coeffs
){

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

    LOG(INFO) << "Pnp:\n" << estimated_T.matrix();

    if(checkEstimatedPose(estimated_T, inliers.rows) == true){

        return true;

    }
    else{
        return false;
    }


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
    std::vector<int> used_idxes;

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

        double z = kp_from_ref_frame->pt3d_.z;

        if(kp_from_ref_frame->pt3d_.z <= 0){
            continue;
        }

        used_idxes.push_back(i);
        pt2ds.push_back(kp_from_current_frame->cv_keypoint_.pt);
        reprojected_pt2ds.push_back(camera2pixel(kp_from_ref_frame->pt3d_, cv_K));
        prev_pt2ds.push_back(kp_from_ref_frame->cv_keypoint_.pt);
        pt3ds.push_back(kp_from_ref_frame->pt3d_);
        eigen_pt2ds.push_back(Eigen::Vector2d(kp_from_current_frame->cv_keypoint_.pt.x, kp_from_current_frame->cv_keypoint_.pt.y));
        eigen_pt3ds.push_back(Eigen::Vector3d(kp_from_ref_frame->pt3d_.x, kp_from_ref_frame->pt3d_.y, kp_from_ref_frame->pt3d_.z));

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


    Sophus::SE3<double> estimated_T_current_cam_ref_cam = Sophus::SE3<double>();
    bool success=false;
    success = PnpEstimator(pt3ds, pt2ds, estimated_T_current_cam_ref_cam, cv_K, cv_distortion_coeffs);
    if(success == false){
        
        camera_frame->status_=CameraFrame::Status::FAIL;
        return;
    }
    img_0_from_current_frame->setTcw(estimated_T_current_cam_ref_cam * img_0_from_ref_frame->T_c_w_);

    LOG(INFO) << "before_BA\n" << img_0_from_current_frame->T_c_w_.matrix();


    // success = bundleAdjustment(estimated_T_current_cam_ref_cam, eigen_pt3ds, eigen_pt2ds, K, distortion_coeffs);
    // if(success == false){
        
    //     camera_frame->status_=CameraFrame::Status::FAIL;
    //     return;
    // }
    
    // // update 3d points after BA
    // for(size_t i=0 ; i<eigen_pt3ds.size();i++){

    //     img_0_from_ref_frame->keypoint_vector_.at(used_idxes[i])->pt3d_ = cv::Point3d(eigen_pt3ds[i](0),eigen_pt3ds[i](1),eigen_pt3ds[i](2));
    // }

    // LOG(INFO) << "after BA\n" << estimated_T_current_cam_ref_cam.matrix();

    // img_0_from_current_frame->setTcw(estimated_T_current_cam_ref_cam * img_0_from_ref_frame->T_c_w_);

    // // LOG(INFO) << "estimated_translation_vec \n" << translation_vec;
    // // LOG(INFO) << "estimated_translation_norm : " << estimated_translation.norm();
    // LOG(INFO) << "current_T_c_w: \n" << img_0_from_current_frame->T_c_w_ .matrix();


    camera_frame->status_=CameraFrame::Status::NORMAL;

}

} // namespace modules_vins

