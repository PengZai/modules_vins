#include "detector.h"


namespace modules_vins{



Detector::Detector(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

    this->orb_feature_ = std::make_shared<ORBFeature>(sys_config->params_->num_feature_points_);
}

void Detector::detect(Image &img){
    
    if(img.data_.channels() == 3){
        cv::cvtColor(img.data_, img.gray_data_, cv::COLOR_BGR2GRAY);
    }
    else if(img.data_.channels() == 1){
        img.gray_data_ = img.data_;
    }
    else{
        VLOG(VERBOSE) << "img is neither the rgb image nor gray image";
        std::exit(EXIT_FAILURE);
    }

    this->orb_feature_->detect(img);


    // VLOG(VERBOSE) << img.data_;
    // VLOG(VERBOSE) << img.gray_data_;



}

void Detector::computeDescriptor(Image &img){

    this->orb_feature_->compute(img);

    VLOG(VERBOSE) << img;
}


void Detector::pipeline(CameraFrame &camera_frame){
    
    for(int i=0; i < (int)camera_frame.image_vector_.size();i++){
        Image &img = camera_frame.image_vector_.at(i);

        detect(img);

        computeDescriptor(img);

        VLOG(VERBOSE) << img;

    }

}
    


}

