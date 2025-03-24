#include "two_view_reconstruction.h"


namespace modules_vins
{


TwoViewReconstructor::TwoViewReconstructor(const std::shared_ptr<SystemConfig> &sys_config){
    this->sys_config_ = sys_config;

}


void TwoViewReconstructor::reconstruct(const std::shared_ptr<Image> &img_i, const std::shared_ptr<Image> &img_j){


    if(img_i->matches_in_frame_.size() == 0){
        VLOG(VERBOSE) << RED <<"There aren't any match information in img i" << RESET;
        return;
    }

    std::vector<cv::Point2f> tracked_points_from_img_i, tracked_points_from_img_j;
    for(int i=0; i < (int)img_i->matches_in_frame_.size(); i++){

        cv::DMatch &match = img_i->matches_in_frame_[i];

        std::shared_ptr<KeyPoint> &tracked_point_from_img_i = img_i->keypoint_vector_[match.queryIdx];

        // if tracked point has been reconstructed in previous point, then we don't reconstruct again,
        // as they belong to same map point.
        if(tracked_point_from_img_i->prev_keypoint_in_time_ != nullptr &&
             tracked_point_from_img_i->prev_keypoint_in_time_->map_point_ptr_!=nullptr){
            // copy the address of the same map point
            tracked_point_from_img_i->map_point_ptr_ = tracked_point_from_img_i->prev_keypoint_in_time_->map_point_ptr_;
            continue;
        }
        tracked_points_from_img_i.push_back(img_i->cv_keypoint_vector_[match.queryIdx].pt);
        tracked_points_from_img_j.push_back(img_j->cv_keypoint_vector_[match.trainIdx].pt);
    }

    cv::Mat points4D;

    // fake current pose
    Eigen::Matrix4d eyeMatrix = Eigen::Matrix4d::Identity();

    Eigen::Matrix<double, 3, 4> P_cam_i_world = this->sys_config_->camera_config_->getProjectionMatrixBetweenCamerasBySensorID(eyeMatrix, img_i->sensor_id_, img_i->sensor_id_);
    Eigen::Matrix<double, 3, 4> P_cam_j_world = this->sys_config_->camera_config_->getProjectionMatrixBetweenCamerasBySensorID(eyeMatrix, img_i->sensor_id_, img_j->sensor_id_);
    
    cv::Mat cv_Pi(3, 4, CV_64F, P_cam_i_world.data());
    cv::Mat cv_Pj(3, 4, CV_64F, P_cam_j_world.data());
    

    cv::triangulatePoints(cv_Pi, cv_Pj, tracked_points_from_img_i, tracked_points_from_img_j, points4D);

    // Step 9: Convert Homogeneous Coordinates to 3D
    for (int i = 0; i < points4D.cols; i++) {
        
        cv::DMatch &match = img_i->matches_in_frame_[i];
        std::shared_ptr<KeyPoint> &keypoint_from_img_i = img_i->keypoint_vector_[match.queryIdx];
        std::shared_ptr<KeyPoint> &keypoint_from_img_j = img_j->keypoint_vector_[match.trainIdx];

        cv::Mat col = points4D.col(i);
        col /= col.at<float>(3);  // Normalize by last coordinate
        cv::Point3f pt3f(col.at<float>(0), col.at<float>(1), col.at<float>(2));
        keypoint_from_img_i->map_point_ptr_ = std::make_shared<MapPoint>(pt3f);
        keypoint_from_img_j->map_point_ptr_ = std::make_shared<MapPoint>(pt3f);

        // objectPoints.emplace_back(col.at<float>(0), col.at<float>(1), col.at<float>(2));
    }

   
 


};



    
} // namespace modules_vins




