#include "initializer.h"


namespace modules_vins{

Initializer::Initializer(const std::shared_ptr<SystemConfig> config):
sys_config_(config)
{

}
    
bool Initializer::initialize(CameraFrame &camera_frame){

    for(int i=0; i < (int)camera_frame.image_vector_.size(); i++){
        std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);
        img_i->initPose();
    }

    return true;
}
    
    
} //namespace modules_vins