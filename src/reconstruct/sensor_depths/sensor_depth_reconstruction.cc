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


    for(int i=0; i < (int)img->keypoint_vector_.size(); i++){

        cv::Point2i &tracked_pt2i_from_img = img->keypoint_vector_[i]->pt2i_;
        
        double depth = img->getPointDepthFromSensor(tracked_pt2i_from_img);

        cv::Point3d img_pt3d = pixel2camera(tracked_pt2i_from_img, depth, cv_K);

        img->keypoint_vector_[i]->pt3d_ = img_pt3d;

    }

    

    // for (int row = 0; row < img->depth_.rows; ++row) {
    //     for (int col = 0; col < img->depth_.cols; ++col) {

    //         cv::Point2d pt = cv::Point2d(col, row);
    //         double depth = img->getPointDepthFromSensor(pt);
    //         img->depth_.at<double>(row, col) = depth;

    //     }
    // }

}

    
} // namespace modules_vins