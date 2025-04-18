#include "detector.h"


namespace modules_vins{



Detector::Detector(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

    this->orb_feature_ = std::make_shared<ORBFeature>(sys_config);
    #ifdef USE_LIBTORCH
    this->yolo_detector_ = std::make_shared<YOLODetector>(sys_config->params_->model_path_ + "/" + sys_config->camera_config_->params_vector_.at(0)->model_name_learned_object_detection_);
    this->yolo_segmentor_ = std::make_shared<YOLOSegmentor>(sys_config->params_->model_path_ + "/" + sys_config->camera_config_->params_vector_.at(0)->model_name_learned_semantic_segmentation_);
    #endif
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

    #ifdef USE_LIBTORCH

    if(this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->use_learned_object_detection_){
        this->yolo_detector_->detect(img);
    }


    if(this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->use_learned_semantic_segmentation_){
        this->yolo_segmentor_->detect(img);

    }
    #endif

    // VLOG(VERBOSE) << img->data_;
    // VLOG(VERBOSE) << img->gray_data_;



}

void Detector::computeDescriptor(const std::shared_ptr<Image> &img){

    this->orb_feature_->compute(img);


}


void Detector::pipeline(std::shared_ptr<CameraFrame> &camera_frame){
    
    if(camera_frame->status_ != CameraFrame::NORMAL){
        return;
    }

    for(int i=0; i < (int)camera_frame->image_vector_.size();i++){
        std::shared_ptr<Image> &img = camera_frame->image_vector_.at(i);

        detect(img);

        computeDescriptor(img);

        // VLOG(VERBOSE) << *img;

    }

}
    


}

