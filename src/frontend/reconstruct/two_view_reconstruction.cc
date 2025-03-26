#include "two_view_reconstruction.h"


namespace modules_vins
{


TwoViewReconstructor::TwoViewReconstructor(const std::shared_ptr<SystemConfig> &sys_config){
    this->sys_config_ = sys_config;

}


// convert keypoint from pixel image plane to normalized camera plane
cv::Point2f TwoViewReconstructor::pixel2norm (const cv::Point2f &pt, const cv::Mat &K ){   

    cv::Point2f noramlized_pt2 = cv::Point2f(
        ( pt.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( pt.y - K.at<double>(1,2) ) / K.at<double>(1,1)  
    );


    return noramlized_pt2;

}


void TwoViewReconstructor::checkTriangulatedPointsWithReprojection(const cv::Point2f &pt2, const cv::Point3f &pt3, const cv::Mat &T, const cv::Mat &K){
    cv::Point2f normalized_pt2 = pixel2norm(pt2, K);

    cv::Mat R = T(cv::Range(0,3), cv::Range(0,3));  // 3x3 rotation
    cv::Mat t = T(cv::Range(0,3), cv::Range(3,4));  // 3x1 translation

    // VLOG(VERBOSE) << "K " << K;
    // VLOG(VERBOSE) << "R " << R;
    // VLOG(VERBOSE) << "t " << t;


    // convert 3d keypoint to camera coordinate;
    cv::Mat reprojected_cam_pt3 = R*(cv::Mat_<double>(3,1) << pt3.x, pt3.y, pt3.z) + t;

    // VLOG(VERBOSE) << "reprojected_cam_pt3 " << reprojected_cam_pt3;

    cv::Point2f reprojected_normalized_pt2(
        reprojected_cam_pt3.at<double>(0,0)/reprojected_cam_pt3.at<double>(2,0), 
        reprojected_cam_pt3.at<double>(1,0)/reprojected_cam_pt3.at<double>(2,0) 
    );

    cv::Point2f res = reprojected_normalized_pt2 - normalized_pt2;

    VLOG(VERBOSE) << " checkTriangulatedPoints ";
    VLOG(VERBOSE) << " pt2 in pixel plane: " << pt2;
    VLOG(VERBOSE) << " pt3 in camera coordinate: " << pt3;
    VLOG(VERBOSE) << " normalized_pt2 :" << normalized_pt2;
    VLOG(VERBOSE) << " reprojected_normalized_pt2: " << reprojected_normalized_pt2;
    VLOG(VERBOSE) << " reprojected_normalized_pt2 - normalized_pt2: " << res;

    
}


void TwoViewReconstructor::reconstruct(const std::shared_ptr<Image> &img_i, const std::shared_ptr<Image> &img_j){


    if(img_i->matches_in_frame_.size() == 0){
        VLOG(VERBOSE) << RED <<"There aren't any match information in img i" << RESET;
        return;
    }

    Eigen::Matrix3d Ki = this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->getIntrinsicsMatrix();
    Eigen::Matrix3d Kj = this->sys_config_->camera_config_->params_vector_.at(img_j->sensor_id_)->getIntrinsicsMatrix();

    cv::Mat cv_Ki(3, 3, CV_64F, Ki.data());
    cv::Mat cv_Kj(3, 3, CV_64F, Kj.data());

    // we collected these tracked points haven't been triangulated yet
    std::vector<cv::Point2f> tracked_kps_from_img_i, tracked_kps_from_img_j;
    std::vector<cv::Point2f> tracked_normalized_kps_from_img_i, tracked_normalized_kps_from_img_j;

    for(int i=0; i < (int)img_i->matches_in_frame_.size(); i++){

        cv::DMatch &match = img_i->matches_in_frame_[i];

        cv::Point2f &tracked_kp_from_img_i = img_i->keypoint_vector_[match.queryIdx]->pt2_;
        cv::Point2f &tracked_kp_from_img_j = img_j->keypoint_vector_[match.trainIdx]->pt2_;

        tracked_normalized_kps_from_img_i.emplace_back(pixel2norm(tracked_kp_from_img_i, cv_Ki));
        tracked_normalized_kps_from_img_j.emplace_back(pixel2norm(tracked_kp_from_img_j, cv_Kj));
    }

    // we can't find any tracked point that could be used for tirangulation
    if(tracked_normalized_kps_from_img_i.size() == 0 || tracked_normalized_kps_from_img_j.size() == 0){
        return;
    }

    cv::Mat points4D;


    Eigen::Matrix<double, 4, 4> T_cam_j_cam_i= this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(img_j->sensor_id_, img_i->sensor_id_);
    Eigen::Matrix<double, 3, 4> Tj = T_cam_j_cam_i.topRows(3);

    // we triangluate 3d point according to image from the zero camera(left camera)
    cv::Mat cv_Ti = (cv::Mat_<double> (3,4) <<
                        1,0,0,0,
                        0,1,0,0,
                        0,0,1,0);

    cv::Mat cv_Tj;
    cv::eigen2cv(Tj, cv_Tj);


    cv::triangulatePoints(cv_Ti, cv_Tj, tracked_normalized_kps_from_img_i, tracked_normalized_kps_from_img_j, points4D);

    // Step 9: Convert Homogeneous Coordinates to 3D
    for (int i = 0; i < points4D.cols; i++) {
        
        cv::DMatch &match = img_i->matches_in_frame_[i];

        cv::Mat col = points4D.col(i);

        col /= col.at<float>(3, 0);  // Normalize by last coordinate
        cv::Point3f img_i_pt3 = cv::Point3f(col.at<float>(0, 0), col.at<float>(1, 0), col.at<float>(2, 0));

        if(this->sys_config_->params_->check_triangulation_){
            cv::Point2f &tracked_kp_from_img_i = img_i->keypoint_vector_[match.queryIdx]->pt2_;
            cv::Point2f &tracked_kp_from_img_j = img_j->keypoint_vector_[match.trainIdx]->pt2_;
    
            checkTriangulatedPointsWithReprojection(tracked_kp_from_img_i, img_i_pt3, cv_Ti, cv_Ki);
            checkTriangulatedPointsWithReprojection(tracked_kp_from_img_j, img_i_pt3, cv_Tj, cv_Kj);
    
        }
      
        // img_i->keypoint_vector_[match.queryIdx]->set3DKeyPoint(img_i_pt3);
        img_i->keypoint_vector_[match.queryIdx]->pt3_ = img_i_pt3;
    }

   
 


}



    
} // namespace modules_vins




