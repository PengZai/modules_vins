#include "detector.h"


namespace modules_vins{



Detector::Detector(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

    this->orb_feature_ = std::make_shared<ORBFeature>(sys_config->params_->num_feature_points_);
}

void Detector::detect(const std::shared_ptr<Image> &img){
    
    if(img->color_data_.channels() == 3){
        cv::cvtColor(img->color_data_, img->gray_data_, cv::COLOR_BGR2GRAY);
    }
    else if(img->color_data_.channels() == 1){
        img->gray_data_ = img->color_data_;
    }
    else{
        VLOG(VERBOSE) << "img is neither the bgr image nor gray image";
        std::exit(EXIT_FAILURE);
    }

    this->orb_feature_->detect(img);


    // VLOG(VERBOSE) << img->data_;
    // VLOG(VERBOSE) << img->gray_data_;



}

void Detector::computeDescriptor(const std::shared_ptr<Image> &img){

    this->orb_feature_->compute(img);


}


void Detector::pipeline(CameraFrame &camera_frame){
    
    for(int i=0; i < (int)camera_frame.image_vector_.size();i++){
        std::shared_ptr<Image> &img = camera_frame.image_vector_.at(i);

        detect(img);

        computeDescriptor(img);

        // VLOG(VERBOSE) << *img;

    }

}
    


}

