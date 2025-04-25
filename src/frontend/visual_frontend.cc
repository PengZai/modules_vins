#include "visual_frontend.h"

namespace modules_vins{


VisualFrontend::VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config),
index_in_camera_frame_deque_for_latest_ref_camera_frame_(0)
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



void VisualFrontend::setRefCameraFrameDeque(const std::deque<std::shared_ptr<CameraFrame>> &ref_camera_frame_deque){

    this->ref_camera_frame_deque_ = ref_camera_frame_deque;
    
}


void VisualFrontend::maintainRefCameraFrameDeque(){

    double culmulative_trans = 0;
    int start_erase_index = -1;
    if(this->ref_camera_frame_deque_.size() > 0){

        double Tcw_translation_norm = this->ref_camera_frame_deque_.front()->image_vector_.at(0)->T_c_w_.translation().norm();
        for(int i=1;i<this->ref_camera_frame_deque_.size();i++){

            const std::shared_ptr<CameraFrame> &prev_camera_frame = this->ref_camera_frame_deque_.at(i);

            const std::shared_ptr<Image> &img_0_from_previous_camera_frame = prev_camera_frame->image_vector_.at(0);
            double prev_Tcw_translation_norm = img_0_from_previous_camera_frame->T_c_w_.translation().norm();
            culmulative_trans  += Tcw_translation_norm - prev_Tcw_translation_norm;
            Tcw_translation_norm = prev_Tcw_translation_norm;

            if(culmulative_trans > this->sys_config_->params_->minimum_cumulative_translation_){
                start_erase_index = i;
                break;
            }
        }

        if(start_erase_index != -1){
            ref_camera_frame_deque_.erase(ref_camera_frame_deque_.begin() + start_erase_index, ref_camera_frame_deque_.end());
        }

    }
    


}



void VisualFrontend::pipeline(std::shared_ptr<CameraFrame> &camera_frame){

  

    this->camera_frame_deque_.push_back(camera_frame);

    VLOG(VERBOSE) << "VisualFrontend Start with camera frame id: " << camera_frame->id_;

    if(camera_frame->id_ == 181){
        VLOG(VERBOSE) << " just test ";
    }

    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    this->detector_->pipeline(camera_frame);
    
    for(int i=0; i<this->ref_camera_frame_deque_.size();i++){

        std::shared_ptr<CameraFrame> &ref_camera_frame = ref_camera_frame_deque_.at(i);
        
        VLOG(VERBOSE) << "estimating pose betweeen reference frame with id: " << ref_camera_frame->id_ << " and current frame with id: " << camera_frame->id_;
        camera_frame->ref_camera_frame_ = ref_camera_frame;

        this->trakcer_->pipeline(camera_frame);
        this->pose_estimator_->pipeline(camera_frame);

        if(camera_frame->status_ == CameraFrame::Status::NORMAL){
            break;
        }
        else{
            camera_frame->cleanTrackInTimeRelationship();
            VLOG(VERBOSE) << YELLOW << " Fail pose estimation with reference camera frame id : " << ref_camera_frame->id_ << " for camera frame id : " << camera_frame->id_ << RESET;
        }

    }

    if(camera_frame->status_ == CameraFrame::Status::NORMAL){
        this->ref_camera_frame_deque_.push_front(camera_frame);
        this->index_in_camera_frame_deque_for_latest_ref_camera_frame_++;
        maintainRefCameraFrameDeque();
    }
    else{
        VLOG(VERBOSE) << YELLOW << " Fail pose estimation for camera frame id : " << camera_frame->id_ << RESET;
        if(camera_frame_deque_.size() - (index_in_camera_frame_deque_for_latest_ref_camera_frame_+1) > this->sys_config_->params_->maximum_num_fail_){

            this->status_ = Status::GET_LOST;
        }
    }

    this->reconstructor_->pipeline(camera_frame);

    VLOG(VERBOSE) << "VisualFrontend End with camera frame id: " << camera_frame->id_;

}


} //modules_vins

