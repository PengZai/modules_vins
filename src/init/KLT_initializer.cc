#include "KLT_initializer.h"



namespace modules_vins{


KLTInitializer::KLTInitializer(const std::shared_ptr<SystemConfig> &sys_config):
Initializer(sys_config)
{

}

void KLTInitializer::pipeline(const std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque){



    const std::shared_ptr<CameraFrame> &camera_frame = camera_frame_deque.back();

    this->data_preprocesor_->pipeline(camera_frame);


    LOG(INFO) << "Initializer start with camera frame id: " << camera_frame->id_;



    const std::shared_ptr<CameraFrame> &latest_camera_frame = camera_frame_deque.back();


    for(int i = (int)this->ref_camera_frame_deque_.size()-1; i>=0; i--){

        const std::shared_ptr<CameraFrame> &ref_camera_frame = this->ref_camera_frame_deque_[i];
        ref_camera_frame->cleanTrackInTimeRelationship();
        latest_camera_frame->ref_camera_frame_ = ref_camera_frame;
        this->tracker_->pipeline(latest_camera_frame);
        this->pose_estimator_->pipeline(latest_camera_frame);

        updateStatus(latest_camera_frame);
        if(this->status_ == Status::SUCCESS){
            break;
        }
   

    }

    if(latest_camera_frame->image_vector_.at(0)->keypoint_vector_.size() < this->sys_config_->params_->num_feature_points_){

        this->detector_->pipeline(latest_camera_frame);
        this->reconstructor_->pipeline(latest_camera_frame);

    }
    this->ref_camera_frame_deque_.push_back(latest_camera_frame);
    if(this->ref_camera_frame_deque_.size()>20){
        this->ref_camera_frame_deque_.pop_front();
    }
    
    this->printfStatus();



    LOG(INFO) << "Initializer end with camera frame id: " << camera_frame->id_;

}


} //namespace modules_vins
