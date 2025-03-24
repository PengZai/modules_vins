#include "visual_frontend.h"

namespace modules_vins{


VisualFrontend::VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{
    
    this->detector_ = std::make_shared<Detector>(sys_config);
    this->trakcer_ = std::make_shared<Tracker>(sys_config);
    this->reconstructor_ = std::make_shared<Reconstructor>(sys_config);

}


void VisualFrontend::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;
}



void VisualFrontend::pipeline(CameraFrame &camera_frame){



    this->detector_->pipeline(camera_frame);
    

    this->trakcer_->pipeline(camera_frame);

    this->reconstructor_->pipeline(camera_frame);


    this->map_->update(camera_frame);

    VLOG(VERBOSE) << "visual frontend end";

    


}


} //modules_vins

