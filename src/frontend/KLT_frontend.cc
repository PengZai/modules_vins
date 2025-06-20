#include "KLT_frontend.h"



namespace modules_vins{


KLTFrontend::KLTFrontend(const std::shared_ptr<SystemConfig> &sys_config):
VisualFrontend(sys_config)
{


    

}


void KLTFrontend::pipeline(std::shared_ptr<CameraFrame> &camera_frame){


    LOG(INFO) << "VisualFrontend Start with camera frame id: " << camera_frame->id_;

    this->data_preprocesor_->pipeline(camera_frame);

    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    
    for(size_t i=0; i<this->ref_camera_frame_deque_.size();i++){

        std::shared_ptr<CameraFrame> &ref_camera_frame = ref_camera_frame_deque_.at(i);
        ref_camera_frame->cleanTrackInTimeRelationship();

        LOG(INFO) << "estimating pose betweeen reference frame with id: " << ref_camera_frame->id_ << " and current frame with id: " << camera_frame->id_;
        camera_frame->ref_camera_frame_ = ref_camera_frame;

        this->tracker_->pipeline(camera_frame);
        this->pose_estimator_->pipeline(camera_frame);

        if(camera_frame->status_ == CameraFrame::Status::NORMAL){
            break;
        }
        else{
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

    if(camera_frame->image_vector_.at(0)->keypoint_vector_.size() < this->sys_config_->params_->num_feature_points_){
        this->detector_->pipeline(camera_frame);
        this->reconstructor_->pipeline(camera_frame);
    }

    LOG(INFO) << "VisualFrontend End with camera frame id: " << camera_frame->id_;

}







} //modules_vins
