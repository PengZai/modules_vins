#include "feature_points.h"

namespace modules_vins{




ORBFeature::ORBFeature(const std::shared_ptr<SystemConfig> &sys_config){

    this->num_features_ = sys_config->params_->num_feature_points_;
    this->scale_factor_ = sys_config->params_->scale_factor_;
    this->level_pyramid_ = sys_config->params_->level_pyramid_;
    this->orb = cv::ORB::create(num_features_, scale_factor_, level_pyramid_);
    
}

void ORBFeature::detect(const std::shared_ptr<Image> &img){


    this->orb->detect(img->color_data_, img->cv_keypoint_vector_);
    for(int i=0; i < (int)img->cv_keypoint_vector_.size(); i++){
        const std::shared_ptr<KeyPoint> &kp = std::make_shared<KeyPoint>(img->cv_keypoint_vector_[i]);
        img->keypoint_vector_.emplace_back(kp);
    }

    
}

void ORBFeature::compute(const std::shared_ptr<Image> &img){

    std::vector<cv::KeyPoint> cvKeypoints;
    this->orb->compute(img->color_data_, img->cv_keypoint_vector_, img->descriptors_);


}
    

FASTFeature::FASTFeature(int num_features){

    this->num_features_ = num_features;
    this->fast = cv::FastFeatureDetector::create(num_features, true);


}

void FASTFeature::detect(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints){


    this->fast->detect(img, keypoints);
}



STCornerFeature::STCornerFeature(int num_features){

        this->num_features_ = num_features;
}


void STCornerFeature::detect(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints){



    // Detect good feature points to track (Shi-Tomasi corner detection)
    // cv::goodFeaturesToTrack(img, keypoints, 100, 0.01, 10);

}
      
    

    

   

} //modules_vins