#include "reconstructor.h"

namespace modules_vins{


Reconstructor::Reconstructor(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;
    this->two_view_reconstructor_ = std::make_shared<TwoViewReconstructor>(sys_config);
    this->sensor_depth_reconstructor_ = std::make_shared<SensorDepthReconstruction>(sys_config);

    #ifdef USE_LIBTORCH
    if(this->sys_config_->camera_config_->params_vector_.at(0)->use_learned_depth_){
        this->midas_reconstructor_ = std::make_shared<MiDas>(sys_config->params_->model_path_ + "/" + sys_config->camera_config_->params_vector_.at(0)->model_name_learned_depth_);
    }
    if(this->sys_config_->camera_config_->params_vector_.at(0)->use_learned_stereo_matching_){

        this->fast_acvnet_plus_reconstructor_ = std::make_shared<FastACVNetPlus>(this->sys_config_, sys_config->params_->model_path_ + "/" + sys_config->camera_config_->params_vector_.at(0)->model_name_learned_stereo_matching_);
        // this->foundation_stereo_reconstructor_ = std::make_shared<FoundationStereo>(this->sys_config_, sys_config->params_->model_path_ + "/" + sys_config->camera_config_->params_vector_.at(0)->model_name_learned_stereo_matching_);

    }

    #endif
}


void Reconstructor::setTracker(const std::shared_ptr<Tracker> &tracker){

    this->tracker_ = tracker;
}

void Reconstructor::pipeline(std::shared_ptr<CameraFrame> &camera_frame){




    // sensor depth reconstruction
    for(int i=0;i<(int)camera_frame->image_vector_.size();i++){

        std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);

        if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_sensor_depth_){
            this->sensor_depth_reconstructor_->reconstruct(img_i);
        }

        #ifdef USE_LIBTORCH
        if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_learned_depth_){
            this->midas_reconstructor_->reconstruct(img_i);
        }
        #endif

    }
    
    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    if(camera_frame->image_vector_.size() > 1){
        this->tracker_->trackInFrame(camera_frame);

        // two view reconstruction
        for(int i=1;i<(int)camera_frame->image_vector_.size();i++){

            std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(1);
            this->two_view_reconstructor_->reconstruct(img_0, img_i);

            if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_stereo_matching_){
                this->two_view_reconstructor_->stereoBatchMatching(img_0, img_i);
            }

            #ifdef USE_LIBTORCH
            if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_learned_stereo_matching_){
                this->fast_acvnet_plus_reconstructor_->reconstruct(img_0, img_i);

                // this->foundation_stereo_reconstructor_->reconstruct(img_0, img_i);
            }
            #endif

        }
    }

    // img_0->sensor_depth_.copyTo(img_0->depth_);  // basic type conversion
    // img_0->stereo_depth_.copyTo(img_0->depth_);  // basic type conversion
    // img_0->sensor_depth_.copyTo(img_0->depth_);  // basic type conversion


    // project 3d point in camera coordinate to map coordinate
    // for(int i=0; i<(int)img_0->keypoint_vector_.size(); i++){

    //     std::shared_ptr<KeyPoint> &kp = img_0->keypoint_vector_.at(i);


    //     if(kp->prev_keypoint_in_time_ == nullptr && kp->pt3d_.z > 0){

    //         Eigen::Vector3d map_point = img_0->T_c_w_.inverse() * Eigen::Vector3d(kp->pt3d_.x, kp->pt3d_.y, kp->pt3d_.z);
    //         std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(map_point);
    //         cv::Vec3b bgr = img_0->color_data_.at<cv::Vec3b>(kp->pt2i_);
    //         map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);

    //         kp->setMapPointPtr(map_point_ptr);
    //         camera_frame->map_point_vector_.emplace_back(map_point_ptr);
            
    //     }
        

    // }

    // cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img_0->sensor_id_)->getCVIntrinsicsMatrix();

    // for (int row = 0; row < img_0->depth_.rows; ++row) {
    //     for (int col = 0; col < img_0->depth_.cols; ++col) {

    //         double depth = img_0->depth_.at<double>(row, col);
    //         if(depth == -1){
    //             // LOG(INFO) << "depth = -1 at" << " row: " << row << ", col: " << col;
    //             continue;
    //         }

    //         cv::Point2i pt2i = cv::Point2i(row, col);
    //         auto it = std::find_if(img_0->keypoint_vector_.begin(), img_0->keypoint_vector_.end(), 
    //         [&](const std::shared_ptr<KeyPoint> &kp){
    //                 return kp->pt2i_ == pt2i;
    //         });

            

    //         // skip all the keypoints, because their mappoints have created
    //         if(it != img_0->keypoint_vector_.end()){
    //             continue;
    //         }

    //         cv::Point3d pt3d = pixel2camera(pt2i, depth, cv_K);

    //         Eigen::Vector3d map_point = img_0->T_c_w_.inverse() * Eigen::Vector3d(pt3d.x, pt3d.y, pt3d.z);
    //         std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(map_point);            
    //         cv::Vec3b bgr = img_0->color_data_.at<cv::Vec3b>(pt2i);
    //         map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);
    //         camera_frame->map_point_vector_.emplace_back(map_point_ptr);


    //     }
    // }
    

}




} //modules_vins