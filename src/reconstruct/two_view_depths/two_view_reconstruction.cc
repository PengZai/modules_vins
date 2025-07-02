#include "two_view_reconstruction.h"


namespace modules_vins
{


TwoViewReconstructor::TwoViewReconstructor(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

    this->num_disparities_ = 64;
    int block_size = 15;
    this->stereoBM_ = cv::StereoBM::create(this->num_disparities_, block_size);

}




void TwoViewReconstructor::checkTriangulatedPointsWithReprojection(const cv::Point2d &pt2d, const cv::Point3d &pt3d, const cv::Mat &T, const cv::Mat &K){
    cv::Point2d normalized_pt2d = pixel2norm(pt2d, K);

    cv::Mat R = T(cv::Range(0,3), cv::Range(0,3));  // 3x3 rotation
    cv::Mat t = T(cv::Range(0,3), cv::Range(3,4));  // 3x1 translation

    // LOG(INFO) << "K " << K;
    // LOG(INFO) << "R " << R;
    // LOG(INFO) << "t " << t;


    // convert 3d keypoint to camera coordinate;
    cv::Mat reprojected_cam_pt3d = R*(cv::Mat_<double>(3,1) << pt3d.x, pt3d.y, pt3d.z) + t;

    // LOG(INFO) << "reprojected_cam_pt3d " << reprojected_cam_pt3d;

    cv::Point2d reprojected_normalized_pt2d(
        reprojected_cam_pt3d.at<double>(0,0)/reprojected_cam_pt3d.at<double>(2,0), 
        reprojected_cam_pt3d.at<double>(1,0)/reprojected_cam_pt3d.at<double>(2,0) 
    );

    cv::Point2d res = reprojected_normalized_pt2d - normalized_pt2d;

    LOG(INFO) << " checkTriangulatedPoints ";
    LOG(INFO) << " pt2d in pixel plane: " << pt2d;
    LOG(INFO) << " pt3d in camera coordinate: " << pt3d;
    LOG(INFO) << " normalized_pt2 :" << normalized_pt2d;
    LOG(INFO) << " reprojected_normalized_pt2d: " << reprojected_normalized_pt2d;
    LOG(INFO) << " reprojected_normalized_pt2d - normalized_pt2: " << res;

    
}




void TwoViewReconstructor::reconstruct(const std::shared_ptr<Image> &img_i, const std::shared_ptr<Image> &img_j){

    if(img_i->matches_in_frame_.size() == 0){
        LOG(INFO) << RED <<"There aren't any match information in img i" << RESET;
        return;
    }

    // twoViewTriangulationWithOpenCV(img_i, img_j);
    twoViewTriangulationWithSVD(img_i, img_j);

    
}


void TwoViewReconstructor::stereoBatchMatching(const std::shared_ptr<Image> &left_img, const std::shared_ptr<Image> &right_img){


    cv::Mat disparity;  

    this->stereoBM_->compute(left_img->gray_data_, right_img->gray_data_, disparity);

    disparity.convertTo(disparity, CV_32FC1, 1.0 / 16.);

    Eigen::Matrix<double, 4, 4> T_right_cam_left_cam= this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(right_img->sensor_id_, left_img->sensor_id_);
    Eigen::Vector3d t = T_right_cam_left_cam.block<3,1>(0, 3);  // Get translation vector
    double B = t.norm();
    const Eigen::Matrix3d K_left = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(left_img->sensor_id_)->getIntrinsicsMatrix();
    double fx = K_left(0,0);

    cv::Mat validMask = disparity > 0;
    cv::Mat depth = cv::Mat::zeros(disparity.size(), CV_32FC1);

    cv::Mat tmp_depth;
    cv::divide(fx * B, disparity, tmp_depth);  // tmp = fx * B / disparity
    tmp_depth.copyTo(depth, validMask);       // apply only where valid

    left_img->stereo_depth_ = depth;


}

/**
 * linear triangulation with SVD
 * s1 * x1 = P * X, s1 is depth scale, x1 is normalized pixel in camera coordinate, P is [R|t], X is world point(actuall point in left camera coordinate)
 * @param poses     poses, the pose could see pt_world
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 * 
 */

bool TwoViewReconstructor::triangulatePoint(const std::vector<Eigen::Matrix<double, 3, 4>> &poses,
                   const std::vector<Eigen::Vector3d> points, Eigen::Vector3d &pt_world) {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(2 * poses.size(), 4);
    Eigen::Matrix<double, Eigen::Dynamic, 1> b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Eigen::Matrix<double, 3, 4> m = poses[i];
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);      // u * P3 - P1
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);  // v * P3 - P2
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    // if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
    //     // solution qualtiy is not good, give up
    //     return true;
    // }

    // return false;
    // std::cout << "condition number : " << svd.singularValues()[0] / svd.singularValues()[3] << std::endl;

    if (svd.singularValues()[3] / svd.singularValues()[2] > 1e-2 
        || pt_world[2] < 0
        || pt_world[2] < this->sys_config_->params_->minimum_estimated_depth_
        || pt_world[2] > this->sys_config_->params_->maximum_estimated_depth_
    ) {
        // solution qualtiy is not good, give up
        return false;
    }

    return true;
}


void TwoViewReconstructor::twoViewTriangulationWithSVD(const std::shared_ptr<Image> &img_i, const std::shared_ptr<Image> &img_j){

    std::vector<Eigen::Matrix<double, 3, 4>> poses;

    cv::Mat cv_Ki = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_i->sensor_id_)->getCVIntrinsicsMatrix();
    cv::Mat cv_Kj = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_j->sensor_id_)->getCVIntrinsicsMatrix();

    Eigen::Matrix<double, 4, 4> T_cam_i_cam_0= this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(img_i->sensor_id_, 0);
    Eigen::Matrix<double, 4, 4> T_cam_j_cam_i= this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(img_j->sensor_id_, img_i->sensor_id_);


    // Tciw = Tcic0 * (Tbc0)^(-1) * * Tbw 
    const Eigen::Matrix<double, 4, 4> &T_cam_i_w = T_cam_i_cam_0 * this->sys_config_->camera_config_->getParamsAt<CameraParameters>(0)->T_base_sensor_.inverse() * img_i->frame_->T_b_w_.matrix();
    const Eigen::Matrix<double, 4, 4> &T_cam_j_w = T_cam_j_cam_i * T_cam_i_w;


    Eigen::Matrix<double, 3, 4> Ti;
    Ti = T_cam_i_w.block<3,4>(0,0);

    Eigen::Matrix<double, 3, 4> Tj = T_cam_j_w.topRows(3);

    poses.emplace_back(Ti);
    poses.emplace_back(Tj);

    int existed_mappoint_count = 0;
    int triangulated_point_count = 0;

    for(size_t i=0; i < img_i->matches_in_frame_.size(); i++){

        std::vector<Eigen::Vector3d> normalized_pt3d, normalized_pt3d_;
        Eigen::Vector3d pt_world, pt_world_;
        cv::DMatch &match = img_i->matches_in_frame_[i];

        const std::shared_ptr<KeyPoint> &kp_from_img_i = img_i->keypoint_vector_[match.queryIdx];
        const std::shared_ptr<KeyPoint> &kp_from_img_j = img_j->keypoint_vector_[match.trainIdx];

        if(kp_from_img_i->map_point_ptr_ != nullptr){

            cv::Vec3b bgr = img_i->color_data_.at<cv::Vec3b>(kp_from_img_i->cv_keypoint_.pt);
            kp_from_img_i->map_point_ptr_->setColor(bgr[0], bgr[1], bgr[2]);
            kp_from_img_j->map_point_ptr_ = kp_from_img_i->map_point_ptr_;

            existed_mappoint_count++;
            continue;
        }

        cv::Point2f &tracked_pt2f_from_img_i = kp_from_img_i->cv_keypoint_.pt;
        cv::Point2f &tracked_pt2f_from_img_j = kp_from_img_j->cv_keypoint_.pt;

        cv::Point2d normalized_pt2d_from_img_i_ = pixel2norm(tracked_pt2f_from_img_i, cv_Ki);
        cv::Point2d normalized_pt2d_from_img_j_ = pixel2norm(tracked_pt2f_from_img_j, cv_Kj);
        
        normalized_pt3d_.emplace_back(Eigen::Vector3d(normalized_pt2d_from_img_i_.x, normalized_pt2d_from_img_i_.y, 1.0));
        normalized_pt3d_.emplace_back(Eigen::Vector3d(normalized_pt2d_from_img_j_.x, normalized_pt2d_from_img_j_.y, 1.0));

        cv::Point2d normalized_pt2d_from_img_i = kp_from_img_i->undistorted_pt2d_;
        cv::Point2d normalized_pt2d_from_img_j = kp_from_img_j->undistorted_pt2d_;     

 
        normalized_pt3d.emplace_back(Eigen::Vector3d(normalized_pt2d_from_img_i.x, normalized_pt2d_from_img_i.y, 1.0));
        normalized_pt3d.emplace_back(Eigen::Vector3d(normalized_pt2d_from_img_j.x, normalized_pt2d_from_img_j.y, 1.0));
        
        // bool success_ = triangulatePoint(poses, normalized_pt3d_, pt_world_);


        bool success = triangulatePoint(poses, normalized_pt3d, pt_world);
        if(success == false){
            continue;
        }

        // img_i->keypoint_vector_[match.queryIdx]->pt3d_ = cv::Point3d(pt_world(0), pt_world(1), pt_world(2));
        // LOG(INFO) << GREEN << "pt3d : " << img_i->keypoint_vector_[match.queryIdx]->pt3d_ << RESET;
        std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(Eigen::Vector3d(pt_world(0), pt_world(1), pt_world(2)));
        // std::shared_ptr<MapPoint> map_point_ptr2 = std::make_shared<MapPoint>(Eigen::Vector3d(pt_world_(0), pt_world_(1), pt_world_(2)));

        cv::Vec3b bgr = img_i->color_data_.at<cv::Vec3b>(kp_from_img_i->cv_keypoint_.pt);
        map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);

        kp_from_img_i->map_point_ptr_ = map_point_ptr;
        kp_from_img_j->map_point_ptr_ = map_point_ptr;
        
        // kp_from_img_i->map_point_ptr2_ = map_point_ptr2;
        // kp_from_img_j->map_point_ptr2_ = map_point_ptr2;

        triangulated_point_count++;

    }

    LOG(INFO) << GREEN << "there are " <<  existed_mappoint_count << " existed mappoints , and " << triangulated_point_count << " triangulated points is successful" << RESET;



}


void TwoViewReconstructor::twoViewTriangulationWithOpenCV(const std::shared_ptr<Image> &img_i, const std::shared_ptr<Image> &img_j){



    cv::Mat cv_Ki = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_i->sensor_id_)->getCVIntrinsicsMatrix();
    cv::Mat cv_Kj = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_j->sensor_id_)->getCVIntrinsicsMatrix();


    // we collected these tracked points haven't been triangulated yet
    std::vector<cv::Point2d> tracked_kps_from_img_i, tracked_kps_from_img_j;
    std::vector<cv::Point2d> tracked_normalized_pt2fs_from_img_i, tracked_normalized_pt2fs_from_img_j;

    for(size_t i=0; i < (int)img_i->matches_in_frame_.size(); i++){

        cv::DMatch &match = img_i->matches_in_frame_[i];

        cv::Point2f &tracked_pt2f_from_img_i = img_i->keypoint_vector_[match.queryIdx]->cv_keypoint_.pt;
        cv::Point2f &tracked_pt2f_from_img_j = img_j->keypoint_vector_[match.trainIdx]->cv_keypoint_.pt;

        tracked_normalized_pt2fs_from_img_i.emplace_back(pixel2norm(tracked_pt2f_from_img_i, cv_Ki));
        tracked_normalized_pt2fs_from_img_j.emplace_back(pixel2norm(tracked_pt2f_from_img_j, cv_Kj));
    }

    // we can't find any tracked point that could be used for tirangulation
    if(tracked_normalized_pt2fs_from_img_i.size() == 0 || tracked_normalized_pt2fs_from_img_j.size() == 0){
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


    cv::triangulatePoints(cv_Ti, cv_Tj, tracked_normalized_pt2fs_from_img_i, tracked_normalized_pt2fs_from_img_j, points4D);

    // Step 9: Convert Homogeneous Coordinates to 3D
    for (int i = 0; i < points4D.cols; i++) {
        
        cv::DMatch &match = img_i->matches_in_frame_[i];

        cv::Mat col = points4D.col(i);

        col /= col.at<double>(3, 0);  // Normalize by last coordinate
        if(col.at<double>(2, 0) <= 0){
            // LOG(INFO) << "keypoint " << match.queryIdx << " has negative z" << col;
            continue;

        }
        cv::Point3d img_i_pt3d = cv::Point3d(col.at<double>(0, 0), col.at<double>(1, 0), col.at<double>(2, 0));

        if(this->sys_config_->params_->check_triangulation_){
            cv::Point2f &tracked_pt2f_from_img_i = img_i->keypoint_vector_[match.queryIdx]->cv_keypoint_.pt;
            cv::Point2f &tracked_pt2f_from_img_j = img_j->keypoint_vector_[match.trainIdx]->cv_keypoint_.pt;
    
            checkTriangulatedPointsWithReprojection(tracked_pt2f_from_img_i, img_i_pt3d, cv_Ti, cv_Ki);
            checkTriangulatedPointsWithReprojection(tracked_pt2f_from_img_j, img_i_pt3d, cv_Tj, cv_Kj);
    
        }
      
        // img_i->keypoint_vector_[match.queryIdx]->set3DKeyPoint(img_i_pt3);
        img_i->keypoint_vector_[match.queryIdx]->pt3d_ = img_i_pt3d;
    }

   
 


}



    
} // namespace modules_vins




