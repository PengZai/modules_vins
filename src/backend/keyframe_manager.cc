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
    double diff_trans = std::abs(frame->T_b_w_.translation().norm() - ref_key_frame->T_b_w_.translation().norm());
    
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


} //namespace modules_vins
