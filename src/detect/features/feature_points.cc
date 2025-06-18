#include "feature_points.h"

namespace modules_vins{


FeaturePoint::FeaturePoint(const std::shared_ptr<SystemConfig> &sys_config){

    this->num_features_ = sys_config->params_->num_feature_points_;
  

}

ORBFeature::ORBFeature(const std::shared_ptr<SystemConfig> &sys_config):
FeaturePoint(sys_config)
{

    this->scale_factor_ = sys_config->params_->scale_factor_;
    this->level_pyramid_ = sys_config->params_->level_pyramid_;
    this->orb_ = cv::ORB::create(this->num_features_, scale_factor_, level_pyramid_);
    this->gftt_ = cv::GFTTDetector::create(this->num_features_, 0.01, 20);
}

void ORBFeature::detect(const std::shared_ptr<Image> &img){
  

    // cv::Mat mask(img->gray_data_.size(), CV_8UC1, 255);

    // for (std::shared_ptr<KeyPoint> &kp : img->keypoint_vector_) {
    //     cv::rectangle(mask, kp->pt2i_ - cv::Point2i(10, 10),
    //                   kp->pt2i_ + cv::Point2i(10, 10), 0, cv::FILLED);
    // }

    int count_new_detected = 0;

    this->gftt_->detect(img->gray_data_, img->cv_keypoint_vector_);
    
    

}




void ORBFeature::compute(const std::shared_ptr<Image> &img){

    std::vector<cv::KeyPoint> cvKeypoints;
    this->orb_->compute(img->gray_data_, img->cv_keypoint_vector_, img->descriptors_);

    for(int i=0; i < (int)img->cv_keypoint_vector_.size(); i++){
        const std::shared_ptr<KeyPoint> &kp = std::make_shared<KeyPoint>(img->cv_keypoint_vector_[i]);
        img->keypoint_vector_.emplace_back(kp);
    }

    LOG(INFO) << " we leave " << img->cv_keypoint_vector_.size() << " features after descriptor computation";

}


    

GoodFeature::GoodFeature(const std::shared_ptr<SystemConfig> &sys_config):
FeaturePoint(sys_config)
{

}


void GoodFeature::detect(const std::shared_ptr<Image> &img){

    this->gftt_->detect(img->gray_data_, img->cv_keypoint_vector_);

}

void GoodFeature::compute(const std::shared_ptr<Image> &img){

}
    

   

} //modules_vins