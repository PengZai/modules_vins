#include "descriptor_match_frontend.h"



namespace modules_vins{


DescriptorMatchFrontend::DescriptorMatchFrontend(const std::shared_ptr<SystemConfig> &sys_config):
VisualFrontend(sys_config)
{


    

}


void DescriptorMatchFrontend::pipeline(std::shared_ptr<CameraFrame> &camera_frame){


    LOG(INFO) << "VisualFrontend Start with camera frame id: " << camera_frame->id_;

    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    this->detector_->pipeline(camera_frame);
    
    for(int i=0; i<this->ref_camera_frame_deque_.size();i++){

        std::shared_ptr<CameraFrame> &ref_camera_frame = ref_camera_frame_deque_.at(i);
        
        LOG(INFO) << "estimating pose betweeen reference frame with id: " << ref_camera_frame->id_ << " and current frame with id: " << camera_frame->id_;
        camera_frame->ref_camera_frame_ = ref_camera_frame;

        this->tracker_->pipeline(camera_frame);
        this->pose_estimator_->pipeline(camera_frame);

        if(camera_frame->status_ == CameraFrame::Status::NORMAL){
            break;
        }
        else{
            camera_frame->cleanTrackInTimeRelationship();
            LOG(INFO) << YELLOW << " Fail pose estimation with reference camera frame id : " << ref_camera_frame->id_ << " for camera frame id : " << camera_frame->id_ << RESET;
        }

    }

    if(camera_frame->status_ == CameraFrame::Status::NORMAL){
        this->ref_camera_frame_deque_.push_front(camera_frame);
        maintainRefCameraFrameDeque();
        this->fail_pose_estimation_num_=0;
    }
    else{
        LOG(INFO) << YELLOW << " Fail pose estimation for camera frame id : " << camera_frame->id_  << ", discard it"<< RESET;
        this->fail_pose_estimation_num_++;
        if(fail_pose_estimation_num_ > this->sys_config_->params_->maximum_num_fail_){

            this->status_ = Status::GET_LOST;
        }
    }

    this->reconstructor_->pipeline(camera_frame);

    LOG(INFO) << "VisualFrontend End with camera frame id: " << camera_frame->id_;

}







} //modules_vins
