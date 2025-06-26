#include "feature_points.h"

namespace modules_vins{


FeaturePoint::FeaturePoint(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;
    this->num_feature_points_ = sys_config->feature_and_tracker_config_->params_->num_feature_points_;
    this->min_distance_ = sys_config->feature_and_tracker_config_->params_->min_distance_;


}



ORBFeature::ORBFeature(const std::shared_ptr<SystemConfig> &sys_config):
FeaturePoint(sys_config)
{

    this->scale_factor_ = sys_config->feature_and_tracker_config_->orb_params_->scale_factor_;
    this->level_pyramid_ = sys_config->feature_and_tracker_config_->orb_params_->level_pyramid_;
    this->orb_ = cv::ORB::create(this->num_feature_points_, scale_factor_, level_pyramid_);
    this->gftt_ = cv::GFTTDetector::create(this->num_feature_points_, 0.01, this->min_distance_*2);
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

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVIntrinsicsMatrix();         
    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVDistortionCoeffs();


    img->appendAndUndistotKeyPoints(cv_key_points, descriptors, cv_K, cv_distortion_coeffs);
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

    this->gftt_ = cv::GFTTDetector::create(this->num_feature_points_, 0.01, this->min_distance_*2);


}


void GoodFeature::detect(const std::shared_ptr<Image> &img){


    std::vector<cv::KeyPoint> cv_key_points;
    // this->gftt_ = cv::GFTTDetector::create(this->num_feature_points_ - img->keypoint_vector_.size(), 0.01, this->min_distance_*2);


    cv::Mat mask(img->gray_data_.size(), CV_8UC1, 255);
    for (auto &kp : img->keypoint_vector_) {
        cv::rectangle(mask, kp->cv_keypoint_.pt - cv::Point2f(this->min_distance_, this->min_distance_),
                      kp->cv_keypoint_.pt + cv::Point2f(this->min_distance_, this->min_distance_), 0, cv::FILLED);
    }

    this->gftt_->detect(img->gray_data_, cv_key_points, mask);

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVIntrinsicsMatrix();         
    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVDistortionCoeffs();

    img->appendAndUndistotKeyPoints(cv_key_points, cv_K, cv_distortion_coeffs);


}


void GoodFeature::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){
    
    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }



    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    // if(img_0->keypoint_vector_.size() < this->num_feature_points_ ){
    //     detect(img_0);
    // }
    
    detect(img_0);


    LOG(INFO) << "Detected " << img_0->keypoint_vector_.size() << " features";

    


}


    

   

} //modules_vins