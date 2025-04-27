#include "keyframe_manager.h"




namespace modules_vins{

 
KeyFrameManager::KeyFrameManager(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

}


bool KeyFrameManager::isKeyCameraKeyFrame(const std::shared_ptr<CameraFrame> &camera_frame){

    if(this->key_camera_frame_vector_.size() == 0){
        return true;
    }

    const std::shared_ptr<CameraFrame> &ref_key_camera_frame = this->key_camera_frame_vector_.back();
    double diff_trans = std::abs(camera_frame->image_vector_.at(0)->T_c_w_.translation().norm() - ref_key_camera_frame->image_vector_.at(0)->T_c_w_.translation().norm());
    
    if(diff_trans > this->sys_config_->params_->minimum_key_camera_frame_translation_){
        return true;
    }
    else{
        return false;
    }

}


void KeyFrameManager::updateKeyFrame(const std::shared_ptr<CameraFrame> &camera_frame){

    if(isKeyCameraKeyFrame(camera_frame)){
        camera_frame->is_key_camera_frame_ = true;
        key_camera_frame_vector_.emplace_back(camera_frame);
    }
    else{
        camera_frame->is_key_camera_frame_ = false;
    }

}


} //namespace modules_vins
