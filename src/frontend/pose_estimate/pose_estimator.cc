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

    VLOG(VERBOSE) << "number of inliers: " << num_inliers;
    VLOG(VERBOSE) << "estimated_Transformation: \n" << Transformation.matrix();
    VLOG(VERBOSE) << "the norm of estimated_Transformation.log(): " << d_norm;
    
    

    if(num_inliers < this->sys_config_->params_->min_inliers_)
    {
        VLOG(VERBOSE) << "reject because inlier is too small: " << num_inliers;
        return false;
    }

    if(d_norm > this->sys_config_->params_->threshold_for_pnp_pose_log_norm_){

        VLOG(VERBOSE) <<"reject because motion is too large: " << d_norm;
        return false;
    }

    return true;
    
}

void PoseEstimator::pipeline(CameraFrame &camera_frame){

    if(camera_frame.status_ != CameraFrame::NORMAL){
        return;
    }

    this->camera_frame_deque_.push_back(camera_frame);

    std::shared_ptr<Image> &img_0_from_current_frame = camera_frame.image_vector_.at(0);

    // we only estimate pose between current frame and previous frame
    if(this->camera_frame_deque_.size()>1){
        CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();
        std::shared_ptr<Image> &img_0_from_previous_frame = previous_camera_frame.image_vector_.at(0);

        std::vector<cv::Point2d> pt2ds;
        std::vector<cv::Point3d> pt3ds;


        for(int i=0; i < (int)img_0_from_current_frame->matches_in_time_.size();i++){

            cv::DMatch &match_in_time = img_0_from_current_frame->matches_in_time_.at(i);
            const std::shared_ptr<KeyPoint> &kp_from_previous_frame = img_0_from_previous_frame->keypoint_vector_.at(match_in_time.trainIdx);
            const std::shared_ptr<KeyPoint> &kp_from_current_frame = img_0_from_current_frame->keypoint_vector_.at(match_in_time.queryIdx);

            if(kp_from_previous_frame->pt3d_.z <= 0){
                continue;
            }

            pt2ds.push_back(kp_from_current_frame->pt2i_);
            pt3ds.push_back(kp_from_previous_frame->pt3d_);

        }


        cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(0)->getCVIntrinsicsMatrix();         
        cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(0)->getCVDistortionCoeffs();
 

        cv::Mat rortation_vec, translation_vec;
        cv::Mat R_cv;
        cv::Mat inliers;


        // PnP pair at least 5 pairs
        // bool success = cv::solvePnP(map_points, keypoints,
        //         cv_K, cv_distortion_coeffs,
        //         rortation_vec, translation_vec, false,
        //         cv::SOLVEPNP_EPNP);

        bool success = cv::solvePnPRansac(
            pt3ds,                 // std::vector<cv::Point3d>
            pt2ds,                  // std::vector<cv::Point2d>
            cv_K,                       // Intrinsic matrix
            cv_distortion_coeffs,       // Distortion coefficients
            rortation_vec,                          // Output: rotation vector
            translation_vec,                          // Output: translation vector
            false,                         // Use extrinsic guess? Usually false
            100,                           // RANSAC iterations
            4.0,                           // Reprojection error threshold (pixels)
            0.99,                          // Confidence
            inliers                       // Output: inlier indices
        );


        
        cv::Rodrigues(rortation_vec, R_cv);


        Eigen::Matrix<double, 3, 3> estimated_rotation;
        Eigen::Vector3d estimated_position;

        cv::cv2eigen(R_cv, estimated_rotation);
        cv::cv2eigen(translation_vec, estimated_position);


        Sophus::SE3<double> estimated_T_current_cam_previous_cam = Sophus::SE3<double>(
            Sophus::SO3<double>(estimated_rotation), estimated_position
        );

        if(checkEstimatedPose(estimated_T_current_cam_previous_cam, inliers.rows) == true){

            img_0_from_current_frame->setTcw(estimated_T_current_cam_previous_cam * img_0_from_previous_frame->T_c_w_);
            VLOG(VERBOSE) << "current_T_c_w: \n" << img_0_from_current_frame->T_c_w_ .matrix();

        }
        else{
            camera_frame.status_ = CameraFrame::FAIL;
            return;
        }

        

    }

    
  



}


} // namespace modules_vins

