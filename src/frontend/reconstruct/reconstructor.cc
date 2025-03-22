#include "reconstructor.h"

namespace modules_vins{


Reconstructor::Reconstructor(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;
    this->two_view_reconstructor_ = std::make_shared<TwoViewReconstructor>(sys_config);
}


void Reconstructor::pipeline(CameraFrame &camera_frame){

    Image &img0 = camera_frame.image_vector_.at(0);

    // two view reconstruction
    for(int i=1;i<camera_frame.image_vector_.size();i++){

        Image &img1 = camera_frame.image_vector_.at(1);
        this->two_view_reconstructor_->reconstruct(img0, img1);
    }



}




} //modules_vins