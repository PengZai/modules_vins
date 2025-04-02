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

        VLOG(VERBOSE) << "number of inliers for PnP " << inliers.rows;

        
        cv::Rodrigues(rortation_vec, R_cv);


        Eigen::Matrix<double, 3, 3> estimated_rotation;
        Eigen::Vector3d estimated_position;

        cv::cv2eigen(R_cv, estimated_rotation);
        cv::cv2eigen(translation_vec, estimated_position);


        Sophus::SE3<double> estimated_T_current_cam_previous_cam = Sophus::SE3<double>(
            Sophus::SO3<double>(estimated_rotation), estimated_position
        );

        Sophus::Vector6d d = estimated_T_current_cam_previous_cam.log();


        VLOG(VERBOSE) << "estimated_T_current_cam_previous_cam: \n" << estimated_T_current_cam_previous_cam.matrix();
        VLOG(VERBOSE) << "the norm of relative T.log() " << d.norm();
        
        
        img_0_from_current_frame->setTcw(estimated_T_current_cam_previous_cam * img_0_from_previous_frame->T_c_w_);

        VLOG(VERBOSE) << "current_T_c_w: \n" << img_0_from_current_frame->T_c_w_ .matrix();

        

        camera_frame_deque_.pop_front();
    }

    
  



}


} // namespace modules_vins

