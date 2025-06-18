#include "descriptor_match_initializer.h"



namespace modules_vins{


DescriptorMatchInitializer::DescriptorMatchInitializer(const std::shared_ptr<SystemConfig> &sys_config):
Initializer(sys_config)
{

}

void DescriptorMatchInitializer::pipeline(std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque){



    std::shared_ptr<CameraFrame> &camera_frame = camera_frame_deque.back();

    LOG(INFO) << "Initializer start with camera frame id: " << camera_frame->id_;


    this->detector_->pipeline(camera_frame);

    std::shared_ptr<CameraFrame> &latest_camera_frame = camera_frame_deque.back();


    for(int i = this->ref_camera_frame_deque_.size()-1; i>=0; i--){

        latest_camera_frame->ref_camera_frame_ = this->ref_camera_frame_deque_[i];
        this->tracker_->pipeline(latest_camera_frame);
        this->pose_estimator_->pipeline(latest_camera_frame);

        updateStatus(latest_camera_frame);
        if(this->status_ == Status::SUCCESS){
            return;
        }

    }

    this->ref_camera_frame_deque_.push_back(latest_camera_frame);
    if(this->ref_camera_frame_deque_.size()>20){
        this->ref_camera_frame_deque_.pop_front();
    }
    
    this->printfStatus();


    this->reconstructor_->pipeline(camera_frame);

    LOG(INFO) << "Initializer end with camera frame id: " << camera_frame->id_;

}


} //namespace modules_vins
