#include "visual_frontend.h"

namespace modules_vins{


VisualFrontend::VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{
    
    this->detector_ = std::make_shared<Detector>(sys_config);
    this->trakcer_ = std::make_shared<Tracker>(sys_config);

}

void VisualFrontend::pipeline(CameraFrame &camera_frame){


    // camera_frame_deque_.push_back(camera_frame);

    detector_->pipeline(camera_frame);
    

    trakcer_->pipeline(camera_frame);

    // camera_frame_deque_.pop_front();
    


}


} //modules_vins

