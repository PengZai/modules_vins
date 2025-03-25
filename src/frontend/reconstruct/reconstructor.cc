#include "reconstructor.h"

namespace modules_vins{


Reconstructor::Reconstructor(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;
    this->two_view_reconstructor_ = std::make_shared<TwoViewReconstructor>(sys_config);
}




void Reconstructor::pipeline(CameraFrame &camera_frame){

    std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);

    // two view reconstruction
    for(int i=1;i<(int)camera_frame.image_vector_.size();i++){

        std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(1);
        this->two_view_reconstructor_->reconstruct(img_0, img_i);
    }



}




} //modules_vins