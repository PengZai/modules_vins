#include "keyframe_manager.h"




namespace modules_vins{

 
KeyFrameManager::KeyFrameManager(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

}


bool KeyFrameManager::isKeyFrame(const std::shared_ptr<Frame> &frame){

    if(this->key_frame_vector_.size() == 0){
        return true;
    }

    const std::shared_ptr<Frame> &ref_key_frame = this->key_frame_vector_.back();
    double diff_trans = (frame->T_b_w_.inverse().translation() - ref_key_frame->T_b_w_.inverse().translation()).norm();
    
    if(diff_trans > this->sys_config_->params_->minimum_key_frame_translation_){
        return true;
    }
    else{
        return false;
    }

}


void KeyFrameManager::updateKeyFrame(const std::shared_ptr<Frame> &frame){

    if(isKeyFrame(frame)){
        frame->is_key_frame_ = true;
        key_frame_vector_.emplace_back(frame);
    }
    else{
        frame->is_key_frame_ = false;
    }

}


void KeyFrameManager::reconstructInKeyFrame(const std::shared_ptr<Frame> &frame){

    if(!frame->is_key_frame_){
        return;
    }

    const std::shared_ptr<Image> img = frame->image_vector_.at(0);
    cv::Mat cv_K = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img->sensor_id_)->getCVIntrinsicsMatrix();
    const Eigen::Matrix<double,4,4> &T_b_c = (this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img->sensor_id_)->T_base_sensor_);
    const Sophus::SE3d &T_w_c = img->frame_->T_b_w_.inverse() * Sophus::SE3d::fitToSE3(T_b_c)  ;

    int existed_mappoint_count = 0;
    int new_sensor_point_count = 0;

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
            Eigen::Vector3d pt3d_in_world = T_w_c.rotationMatrix() * Eigen::Vector3d(pt3d_in_cam.x, pt3d_in_cam.y, pt3d_in_cam.z) + T_w_c.translation();
            
            std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(pt3d_in_world);
            // std::shared_ptr<MapPoint> map_point_ptr2 = std::make_shared<MapPoint>(Eigen::Vector3d(pt_world_(0), pt_world_(1), pt_world_(2)));

            cv::Vec3b bgr = img->color_data_.at<cv::Vec3b>(pt2f);
            map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);

            img->mappoint_vector_.emplace_back(map_point_ptr);

            new_sensor_point_count++;

        }
    }

    LOG(INFO) << GREEN << "there are " <<  existed_mappoint_count << " existed mappoints , and " << new_sensor_point_count << " new sensor points are taken as map points successful" << RESET;

}


} //namespace modules_vins
