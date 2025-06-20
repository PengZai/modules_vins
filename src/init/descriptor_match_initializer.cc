#include "descriptor_match_initializer.h"



namespace modules_vins{


DescriptorMatchInitializer::DescriptorMatchInitializer(const std::shared_ptr<SystemConfig> &sys_config):
Initializer(sys_config)
{

}

void DescriptorMatchInitializer::pipeline(const std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque){





    const std::shared_ptr<CameraFrame> &latest_camera_frame = camera_frame_deque.back();

    LOG(INFO) << "Initializer start with camera frame id: " << latest_camera_frame->id_;

    this->data_preprocesor_->pipeline(latest_camera_frame);



    this->detector_->pipeline(latest_camera_frame);




    for(int i = (int)this->ref_camera_frame_deque_.size()-1; i>=0; i--){


        const std::shared_ptr<CameraFrame> &ref_camera_frame = this->ref_camera_frame_deque_[i];

        latest_camera_frame->ref_camera_frame_ = ref_camera_frame;
        ref_camera_frame->cleanTrackInTimeRelationship();

        this->tracker_->pipeline(latest_camera_frame);
        this->pose_estimator_->pipeline(latest_camera_frame);

        updateStatus(latest_camera_frame);
        if(this->status_ == Status::SUCCESS){
            break;
        }

    }

    this->reconstructor_->pipeline(latest_camera_frame);


    this->ref_camera_frame_deque_.push_back(latest_camera_frame);
    if(this->ref_camera_frame_deque_.size()>20){
        this->ref_camera_frame_deque_.pop_front();
    }
    
    this->printfStatus();



    LOG(INFO) << "Initializer end with camera frame id: " << latest_camera_frame->id_;

}


} //namespace modules_vins
