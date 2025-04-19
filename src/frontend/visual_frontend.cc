#include "visual_frontend.h"

namespace modules_vins{


VisualFrontend::VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{
    
    this->detector_ = std::make_shared<Detector>(sys_config);
    this->trakcer_ = std::make_shared<Tracker>(sys_config);
    this->reconstructor_ = std::make_shared<Reconstructor>(sys_config);
    this->pose_estimator_ = std::make_shared<PoseEstimator>(sys_config);

}


VisualFrontend::Status VisualFrontend::getStatus(){

    return this->status_;
}

void VisualFrontend::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;
}






void VisualFrontend::pipeline(std::shared_ptr<CameraFrame> &camera_frame){

    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }

    this->camera_frame_deque_.push_back(camera_frame);

    VLOG(VERBOSE) << "VisualFrontend Start with camera frame id: " << camera_frame->id_;

    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    this->detector_->pipeline(camera_frame);
    

    ref_camera_frame_ = this->camera_frame_deque_.front();

    this->trakcer_->pipeline(ref_camera_frame_, camera_frame);

    this->pose_estimator_->pipeline(ref_camera_frame_, camera_frame);

    this->reconstructor_->pipeline(camera_frame);

  
    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        if(this->camera_frame_deque_.size()>1){
            this->camera_frame_deque_.pop_back();
        }
    }
    else{
        if(this->camera_frame_deque_.size()>1){
            this->camera_frame_deque_.pop_front();
        }
    }


    

    VLOG(VERBOSE) << "VisualFrontend End with camera frame id: " << camera_frame->id_;
    


}


} //modules_vins

