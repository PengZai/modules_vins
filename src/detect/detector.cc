#include "detector.h"


namespace modules_vins{



Detector::Detector(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{

    #ifdef USE_LIBTORCH
    this->yolo_detector_ = std::make_shared<YOLODetector>(sys_config->params_->model_path_ + "/" + sys_config->camera_config_->getParamsAt<CameraParameters>(0)->model_name_learned_object_detection_);
    this->yolo_segmentor_ = std::make_shared<YOLOSegmentor>(sys_config->params_->model_path_ + "/" + sys_config->camera_config_->getParamsAt<CameraParameters>(0)->model_name_learned_semantic_segmentation_);
    #endif
}


void Detector::setFeaturePoint(const std::shared_ptr<FeaturePoint> &feature_point){


    this->feature_point_ = feature_point;

}

void Detector::detect(const std::shared_ptr<Image> &img){
    

    #ifdef USE_LIBTORCH

    if(this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img->sensor_id_)->use_learned_object_detection_){
        this->yolo_detector_->detect(img);
    }


    if(this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img->sensor_id_)->use_learned_semantic_segmentation_){
        this->yolo_segmentor_->detect(img);

    }
    #endif

    // LOG(INFO) << img->gray_data_;



}




void Detector::pipeline(const std::shared_ptr<Frame> &frame){
    
    if(frame->status_ != Frame::Status::NORMAL){
        return;
    }

    for(size_t i=0;i<frame->image_vector_.size();i++){
        
        const std::shared_ptr<Image> &img_i = frame->image_vector_.at(i);

        detect(img_i);

    }
    

    this->feature_point_->pipeline(frame);
    




}
    


}

