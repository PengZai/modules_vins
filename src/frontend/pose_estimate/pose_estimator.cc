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

        std::vector<cv::Point2f> keypoints;
        std::vector<cv::Point3f> map_points;


        for(int i=0; i < (int)img_0_from_previous_frame->matches_in_frame_.size();i++){

            cv::DMatch &match_in_frame = img_0_from_previous_frame->matches_in_frame_.at(i);
            const std::shared_ptr<KeyPoint> &kp_from_previous_frame = img_0_from_previous_frame->keypoint_vector_.at(match_in_frame.queryIdx);
            if(kp_from_previous_frame->next_keypoint_in_time_ == nullptr){
                continue;
            }

            keypoints.push_back(kp_from_previous_frame->next_keypoint_in_time_->pt2_);
            map_points.push_back(kp_from_previous_frame->pt3_);

        }


        Eigen::Matrix3d K = this->sys_config_->camera_config_->params_vector_.at(0)->getIntrinsicsMatrix();
        Eigen::VectorXd distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(0)->getDistortionCoeffs();


        cv::Mat cv_K;
        cv::eigen2cv(K, cv_K); 
         
        cv::Mat cv_distortion_coeffs;
        cv::eigen2cv(distortion_coeffs, cv_distortion_coeffs); 
 

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
            8.0,                           // Reprojection error threshold (pixels)
            0.99,                          // Confidence
            inliers                       // Output: inlier indices
        );

        VLOG(VERBOSE) << "number of inliers for PnP " << inliers.rows;

        
        cv::Rodrigues(rortation_vec, R_cv);


        Eigen::Matrix<double, 3, 3> estimated_rotation;
        Eigen::Vector3d estimated_position;

        cv::cv2eigen(R_cv, estimated_rotation);
        cv::cv2eigen(translation_vec, estimated_position);


        img_0_from_current_frame->rotation_ = estimated_rotation * img_0_from_previous_frame->rotation_;
        img_0_from_current_frame->position_ = estimated_position + img_0_from_previous_frame->position_;

        

        camera_frame_deque_.pop_front();
    }

    
  



}


} // namespace modules_vins

