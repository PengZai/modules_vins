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
    //     cv::rectangle(mask, kp->pt2i_ - cv::Point2f(10, 10),
    //                   kp->pt2i_ + cv::Point2f(10, 10), 0, cv::FILLED);
    // }

    std::vector<cv::KeyPoint> cv_key_points;

    this->gftt_->detect(img->gray_data_, cv_key_points);
    
    cv::Mat descriptors;
    this->orb_->compute(img->gray_data_, cv_key_points, descriptors);

    img->setKeyPoints(cv_key_points, descriptors);

    LOG(INFO) << " we leave " << img->keypoint_vector_.size() << " features after descriptor computation";

    

}



void ORBFeature::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){
    
    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }


    for(size_t i=0;i<camera_frame->image_vector_.size();i++){

        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);
        detect(img_i);

        LOG(INFO) << "Detect " << img_i->keypoint_vector_.size() << " new features";


    }


}




    

GoodFeature::GoodFeature(const std::shared_ptr<SystemConfig> &sys_config):
FeaturePoint(sys_config)
{

}


void GoodFeature::detect(const std::shared_ptr<Image> &img){


    std::vector<cv::KeyPoint> cv_key_points;
    this->gftt_ = cv::GFTTDetector::create(this->num_features_-img->keypoint_vector_.size(), 0.01, 20);

    cv::Mat mask(img->gray_data_.size(), CV_8UC1, 255);
    for (auto &kp : img->keypoint_vector_) {
        cv::rectangle(mask, kp->cv_keypoint_.pt - cv::Point2f(10, 10),
                      kp->cv_keypoint_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }

    this->gftt_->detect(img->gray_data_, cv_key_points, mask);

    img->setKeyPoints(cv_key_points);


}


void GoodFeature::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){
    
    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }



    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);
    detect(img_0);
    
    LOG(INFO) << "Detect " << img_0->keypoint_vector_.size() << " new features";

    


}


    

   

} //modules_vins