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

int PoseEstimator::PnpEstimator(
    const std::vector<cv::Point3d> &pt3ds, 
    const std::vector<cv::Point2d> &pt2ds, 
    cv::Mat &cv_R, 
    cv::Mat &translation_vec,
    const cv::Mat &cv_K,
    const cv::Mat &cv_distortion_coeffs
){

    cv::Mat inliers;
    cv::Mat rortation_vec;

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

    cv::Rodrigues(rortation_vec, cv_R);


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



void PoseEstimator::pipeline(std::shared_ptr<CameraFrame> &ref_camera_frame, std::shared_ptr<CameraFrame> &camera_frame){

    if(camera_frame->status_ != CameraFrame::NORMAL){
        return;
    }


    std::shared_ptr<Image> &img_0_from_current_frame = camera_frame->image_vector_.at(0);

    // we only estimate pose between current frame and previous frame
    std::shared_ptr<Image> &img_0_from_ref_frame = ref_camera_frame->image_vector_.at(0);

    std::vector<cv::Point2d> pt2ds;
    std::vector<cv::Point2d> reprojected_pt2is;
    std::vector<cv::Point2d> prev_pt2ds;
    std::vector<cv::Point3d> pt3ds;

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(0)->getCVIntrinsicsMatrix();         



    for(int i=0; i < (int)img_0_from_current_frame->matches_in_time_.size();i++){

        cv::DMatch &match_in_time = img_0_from_current_frame->matches_in_time_.at(i);
        const std::shared_ptr<KeyPoint> &kp_from_ref_frame = img_0_from_ref_frame->keypoint_vector_.at(match_in_time.trainIdx);
        const std::shared_ptr<KeyPoint> &kp_from_current_frame = img_0_from_current_frame->keypoint_vector_.at(match_in_time.queryIdx);

        double z = kp_from_ref_frame->pt3d_.z;

        if(kp_from_ref_frame->pt3d_.z <= 0){
            continue;
        }

        pt2ds.push_back(kp_from_current_frame->pt2i_);
        reprojected_pt2is.push_back(camera2pixel(kp_from_ref_frame->pt3d_, cv_K));
        prev_pt2ds.push_back(kp_from_ref_frame->pt2i_);
        pt3ds.push_back(kp_from_ref_frame->pt3d_);

    }

    // for(int j=0;j<pt3ds.size();j++){
    //     VLOG(VERBOSE) << "prev:" << prev_pt2is.at(j).x << " projected:"<< reprojected_pt2is.at(j).x << ":" << "curr" << pt2is.at(j).x;
    // }

    // checkImages(img_0_from_previous_frame, img_0_from_current_frame);

    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(0)->getCVDistortionCoeffs();


    cv::Mat translation_vec;
    cv::Mat cv_R;

    // pnp estimator
    // cv::Mat inliers;



    int inlier_num = PnpEstimator(pt3ds, 
        pt2ds, 
        cv_R, 
        translation_vec,
        cv_K,
        cv_distortion_coeffs);

    // int inlier_num = epipolarGeometryEstimator(prev_pt2ds, 
    //     pt2ds, 
    //     cv_R, 
    //     translation_vec,
    //     cv_K);


    
    Eigen::Matrix<double, 3, 3> estimated_rotation;
    Eigen::Vector3d estimated_position;

    cv::cv2eigen(cv_R, estimated_rotation);
    cv::cv2eigen(translation_vec, estimated_position);

    Sophus::SE3<double> estimated_T_current_cam_ref_cam = Sophus::SE3<double>(
        Sophus::SO3<double>(estimated_rotation), estimated_position
    );

    // VLOG(VERBOSE) << "number of pair points: " << pt2ds.size();
    // checkEstimatedPose(estimated_T_current_cam_previous_cam, inlier_num);
    // img_0_from_current_frame->setTcw(estimated_T_current_cam_previous_cam * img_0_from_previous_frame->T_c_w_);

    // VLOG(VERBOSE) << "translation_vec \n" << translation_vec;
    // VLOG(VERBOSE) << "current_T_c_w: \n" << img_0_from_current_frame->T_c_w_ .matrix();

    if(checkEstimatedPose(estimated_T_current_cam_ref_cam, inlier_num) == true){

        img_0_from_current_frame->setTcw(estimated_T_current_cam_ref_cam * img_0_from_ref_frame->T_c_w_);

        VLOG(VERBOSE) << "translation_vec \n" << translation_vec;
        VLOG(VERBOSE) << "current_T_c_w: \n" << img_0_from_current_frame->T_c_w_ .matrix();

    }
    else{
        camera_frame->status_ = CameraFrame::FAIL;
        return;
    }




}


} // namespace modules_vins

