#include "pose_estimator.h"



namespace modules_vins
{
    

PoseEstimator::PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

}


void PoseEstimator::pipeline(CameraFrame &camera_frame){


    std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);

    if(img_0->is_pose_estimated_ == true){
        return;
    }

    std::vector<cv::Point3f> map_points;
    std::vector<cv::Point2f> keypoints;

    for(int i=0; i < (int)img_0->matches_in_time_.size();i++){

        cv::DMatch &match = img_0->matches_in_time_.at(i);
        const std::shared_ptr<KeyPoint> &kp = img_0->keypoint_vector_.at(match.queryIdx);
        if(kp->prev_keypoint_in_time_ == nullptr || kp->prev_keypoint_in_time_->map_point_ptr_ == nullptr){
            continue;
        }

        keypoints.push_back(kp->pt_);
        map_points.push_back(kp->prev_keypoint_in_time_->map_point_ptr_->pt_);
      
    }


    Eigen::Matrix3d K = this->sys_config_->camera_config_->params_vector_.at(0)->getIntrinsicsMatrix();
    Eigen::VectorXd distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(0)->getDistortionCoeffs();


    cv::Mat cv_K(3, 3, CV_64F, K.data());
    cv::Mat cv_distortion_coeffs(4, 1, CV_64F, distortion_coeffs.data());


    cv::Mat rvec, tvec;

    // PnP pair at least 5 pairs
    bool success = cv::solvePnP(map_points, keypoints,
            cv_K, cv_distortion_coeffs,
            rvec, tvec, false,
            cv::SOLVEPNP_EPNP);


    cv::cv2eigen(rvec, img_0->rotation_);
    cv::cv2eigen(tvec, img_0->position_);
    img_0->is_pose_estimated_ = true;
  



}


} // namespace modules_vins

