#include "pose_estimator.h"



namespace modules_vins
{
    

PoseEstimator::PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

}


void PoseEstimator::pipeline(CameraFrame &camera_frame){


    this->camera_frame_deque_.push_back(camera_frame);

    std::shared_ptr<Image> &img_0_from_current_frame = camera_frame.image_vector_.at(0);

    // we only estimate pose between current frame and previous frame
    if(this->camera_frame_deque_.size()>1){
        CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();
        std::shared_ptr<Image> &img_0_from_previous_frame = previous_camera_frame.image_vector_.at(0);

        std::vector<cv::Point3f> map_points;
        std::vector<cv::Point2f> keypoints;

        for(int i=0; i < (int)img_0_from_current_frame->matches_in_time_.size();i++){

            cv::DMatch &match = img_0_from_current_frame->matches_in_time_.at(i);
            const std::shared_ptr<KeyPoint> &kp_from_current_frame = img_0_from_current_frame->keypoint_vector_.at(match.queryIdx);
            const std::shared_ptr<KeyPoint> &kp_from_previous_frame = img_0_from_previous_frame->keypoint_vector_.at(match.trainIdx);

            if(kp_from_previous_frame->map_point_ptr_ == nullptr){
                continue;
            }

            keypoints.push_back(kp_from_current_frame->pt_);
            map_points.push_back(kp_from_previous_frame->map_point_ptr_->pt_);
        
        }


        Eigen::Matrix3d K = this->sys_config_->camera_config_->params_vector_.at(0)->getIntrinsicsMatrix();
        Eigen::VectorXd distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(0)->getDistortionCoeffs();


        cv::Mat cv_K(3, 3, CV_64F, K.data());
        cv::Mat cv_distortion_coeffs(4, 1, CV_64F, distortion_coeffs.data());


        cv::Mat rortation_vec, translation_vec;
        cv::Mat R_cv;
        cv::Mat inliers;


        // PnP pair at least 5 pairs
        // bool success = cv::solvePnP(map_points, keypoints,
        //         cv_K, cv_distortion_coeffs,
        //         rortation_vec, translation_vec, false,
        //         cv::SOLVEPNP_EPNP);

        bool success = cv::solvePnPRansac(
            map_points,                 // std::vector<cv::Point3f>
            keypoints,                  // std::vector<cv::Point2f>
            cv_K,                       // Intrinsic matrix
            cv_distortion_coeffs,       // Distortion coefficients
            rortation_vec,                          // Output: rotation vector
            translation_vec,                          // Output: translation vector
            false,                         // Use extrinsic guess? Usually false
            100,                           // RANSAC iterations
            4.0,                           // Reprojection error threshold (pixels)
            0.99,                          // Confidence
            inliers,                       // Output: inlier indices
            cv::SOLVEPNP_EPNP              // 💡 Algorithm choice here
        );


        // Eigen::Matrix<double, 3, 3> relative_rotation;
        // Eigen::Vector3d relative_position;
        
        cv::Rodrigues(rortation_vec, R_cv);


        cv::cv2eigen(R_cv, img_0_from_current_frame->rotation_);
        cv::cv2eigen(translation_vec, img_0_from_current_frame->position_);

        
   


    }

    
  



}


} // namespace modules_vins

