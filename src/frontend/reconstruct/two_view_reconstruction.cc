#include "two_view_reconstruction.h"


namespace modules_vins
{


TwoViewReconstructor::TwoViewReconstructor(const std::shared_ptr<SystemConfig> &sys_config){
    this->sys_config_ = sys_config;

}


std::vector<cv::Point2f> TwoViewReconstructor::pixel2cam (const std::vector<cv::Point2f> &pts, const Eigen::Matrix3d & K ){   

    std::vector<cv::Point2f> cam_Point2f;

    for(int i=0;i<(int)pts.size();i++){
        const cv::Point2f &p = pts.at(i);
        cam_Point2f.emplace_back(cv::Point2f(
            ( p.x - K(0,2) ) / K(0,0), 
            ( p.y - K(1,2) ) / K(1,1)  
        ));
    }

    return cam_Point2f;

}


void TwoViewReconstructor::reconstruct(const std::shared_ptr<Image> &img_i, const std::shared_ptr<Image> &img_j){


    if(img_i->matches_in_frame_.size() == 0){
        VLOG(VERBOSE) << RED <<"There aren't any match information in img i" << RESET;
        return;
    }

    // we collected these tracked points haven't been triangulated yet
    std::vector<cv::Point2f> not_yet_triangulated_points_from_img_i, not_yet_triangulated_points_from_img_j;
    std::vector<cv::DMatch> not_yet_triangulated_matches;

    for(int i=0; i < (int)img_i->matches_in_frame_.size(); i++){

        cv::DMatch &match = img_i->matches_in_frame_[i];

        std::shared_ptr<KeyPoint> &tracked_point_from_img_i = img_i->keypoint_vector_[match.queryIdx];
        std::shared_ptr<KeyPoint> &tracked_point_from_img_j = img_j->keypoint_vector_[match.trainIdx];

        // if tracked point has been reconstructed in previous point, then we don't reconstruct again,
        // as they belong to same map point.
        if(tracked_point_from_img_i->prev_keypoint_in_time_ != nullptr &&
             tracked_point_from_img_i->prev_keypoint_in_time_->map_point_ptr_!=nullptr){
            // copy the address of the same map point
            tracked_point_from_img_i->setMapPointPtr(tracked_point_from_img_i->prev_keypoint_in_time_->map_point_ptr_);
            tracked_point_from_img_j->setMapPointPtr(tracked_point_from_img_i->map_point_ptr_);
            continue;
        }

        not_yet_triangulated_points_from_img_i.push_back(img_i->cv_keypoint_vector_[match.queryIdx].pt);
        not_yet_triangulated_points_from_img_j.push_back(img_j->cv_keypoint_vector_[match.trainIdx].pt);
        not_yet_triangulated_matches.push_back(match);
    }

    // we can't find any tracked point that could be used for tirangulation
    if(not_yet_triangulated_points_from_img_i.size() == 0 || not_yet_triangulated_points_from_img_j.size() == 0){
        return;
    }

    cv::Mat points4D;

    // fake current pose
    Eigen::Matrix4d T_cam_i_world = Eigen::Matrix4d::Identity();
    T_cam_i_world.block<3,3>(0,0) = img_i->rotation_;
    T_cam_i_world.block<3,1>(0,3) = img_i->position_;

    Eigen::Matrix3d K_i = this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->getIntrinsicsMatrix();
    Eigen::Matrix3d K_j = this->sys_config_->camera_config_->params_vector_.at(img_j->sensor_id_)->getIntrinsicsMatrix();

    Eigen::Matrix<double, 4, 4> T_cam_j_cam_i = this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(img_i->sensor_id_, img_j->sensor_id_);
    Eigen::Matrix<double, 4, 4> T_cam_j_world = T_cam_j_cam_i * T_cam_i_world;


    Eigen::Matrix<double, 3, 4> Ti = T_cam_i_world.topRows(3);
    Eigen::Matrix<double, 3, 4> Tj = T_cam_j_world.topRows(3);


    cv::Mat cv_Ti(3, 4, CV_64F, Ti.data());
    cv::Mat cv_Tj(3, 4, CV_64F, Tj.data());

    std::vector<cv::Point2f> cam_points_from_img_i = pixel2cam(not_yet_triangulated_points_from_img_i, K_i);
    std::vector<cv::Point2f> cam_points_from_img_j = pixel2cam(not_yet_triangulated_points_from_img_j, K_j);


    cv::triangulatePoints(cv_Ti, cv_Tj, cam_points_from_img_i, cam_points_from_img_j, points4D);

    // Step 9: Convert Homogeneous Coordinates to 3D
    for (int i = 0; i < points4D.cols; i++) {
        
        cv::DMatch &match = not_yet_triangulated_matches[i];
        std::shared_ptr<KeyPoint> &keypoint_from_img_i = img_i->keypoint_vector_[match.queryIdx];
        std::shared_ptr<KeyPoint> &keypoint_from_img_j = img_j->keypoint_vector_[match.trainIdx];

        cv::Mat col = points4D.col(i);
        col /= col.at<float>(3, 0);  // Normalize by last coordinate
        cv::Point3f pt3f(col.at<float>(0, 0), col.at<float>(1, 0), col.at<float>(2, 0));

        keypoint_from_img_i->setMapPointPtr(std::make_shared<MapPoint>(pt3f));
        keypoint_from_img_j->setMapPointPtr(std::make_shared<MapPoint>(pt3f));

    }

   
 


}



    
} // namespace modules_vins




