#include "sensor_depth_reconstruction.h"


namespace modules_vins
{



SensorDepthReconstruction::SensorDepthReconstruction(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

}


void SensorDepthReconstruction::reconstruct(const std::shared_ptr<Image> &img){

    // we reconstruct geometry of keypoint in camera coordinate with sensor detph

    
    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVIntrinsicsMatrix();
    const Sophus::SE3d &T_c_w = img->T_c_w_;

    for (int row = 0; row < img->color_data_.rows; ++row) {
        for (int col = 0; col < img->color_data_.cols; ++col) {

            cv::Point2f pt = cv::Point2d(col, row);
            double depth = img->getPointDepthFromSensor(pt);
            img->depth_.at<double>(row, col) = depth;

        }
    }

    int existed_mappoint_count = 0;
    int new_sensor_point_count = 0;


    for(size_t i=0; i < img->keypoint_vector_.size(); i++){

        const std::shared_ptr<KeyPoint> &kp = img->keypoint_vector_[i];
        cv::Point2f &tracked_pt2f_from_img = kp->cv_keypoint_.pt;
        
        if(kp->map_point_ptr_ != nullptr) {
            existed_mappoint_count++;
            continue;
        }

        double depth = img->depth_.at<double>(tracked_pt2f_from_img.y, tracked_pt2f_from_img.x);
        if(depth <= 0 || 
        depth < this->sys_config_->params_->minimum_estimated_depth_ ||
        depth > this->sys_config_->params_->maximum_estimated_depth_){
            continue;
        }

        cv::Point3d pt3d_in_cam = pixel2camera(tracked_pt2f_from_img, depth, cv_K);
        Sophus::SE3d T_w_c = img->T_c_w_.inverse();
        Eigen::Vector3d pt3d_in_world = T_w_c.rotationMatrix() * Eigen::Vector3d(pt3d_in_cam.x, pt3d_in_cam.y, pt3d_in_cam.z) + T_w_c.translation();
        
        std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(pt3d_in_world);
        // std::shared_ptr<MapPoint> map_point_ptr2 = std::make_shared<MapPoint>(Eigen::Vector3d(pt_world_(0), pt_world_(1), pt_world_(2)));

        cv::Vec3b bgr = img->color_data_.at<cv::Vec3b>(kp->cv_keypoint_.pt);
        map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);

        kp->map_point_ptr_ = map_point_ptr;
        new_sensor_point_count++;
    }

    for (int row = 0; row < img->color_data_.rows; ++row) {
        for (int col = 0; col < img->color_data_.cols; ++col) {

            
            double depth = img->depth_.at<double>(row, col);
            if(depth <= 0 ||
            depth < this->sys_config_->params_->minimum_estimated_depth_ ||
            depth > this->sys_config_->params_->maximum_estimated_depth_){
                continue;
            }
            cv::Point2f pt2f = cv::Point2f(col, row);
            cv::Point3d pt3d_in_cam = pixel2camera(pt2f, depth, cv_K);
            Sophus::SE3d T_w_c = img->T_c_w_.inverse();
            Eigen::Vector3d pt3d_in_world = T_w_c.rotationMatrix() * Eigen::Vector3d(pt3d_in_cam.x, pt3d_in_cam.y, pt3d_in_cam.z) + T_w_c.translation();
            
            std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(pt3d_in_world);
            // std::shared_ptr<MapPoint> map_point_ptr2 = std::make_shared<MapPoint>(Eigen::Vector3d(pt_world_(0), pt_world_(1), pt_world_(2)));

            cv::Vec3b bgr = img->color_data_.at<cv::Vec3b>(pt2f);
            map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);

            new_sensor_point_count++;
            img->mappoint_vector_.emplace_back(map_point_ptr);

        }
    }

    
   
    LOG(INFO) << GREEN << "there are " <<  existed_mappoint_count << " existed mappoints , and " << new_sensor_point_count << " new sensor points are taken as map points successful" << RESET;



}

    
} // namespace modules_vins