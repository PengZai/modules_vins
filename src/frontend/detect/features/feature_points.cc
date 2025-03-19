#include "feature_points.h"

namespace modules_vins{




ORBFeature::ORBFeature(int num_features){
    this->orb = cv::ORB::create(num_features);
    
}

void ORBFeature::detect(Image &img){

    this->orb->detect(img.data_, img.cv_keypoint_vector_);
    for(int i=0; i < (int)img.cv_keypoint_vector_.size(); i++){
        cv::KeyPoint &kp = img.cv_keypoint_vector_[i];
        img.keypoint_vector_.emplace_back(kp);
    }

    
}

void ORBFeature::compute(Image &img){

    std::vector<cv::KeyPoint> cvKeypoints;
    this->orb->compute(img.data_, img.cv_keypoint_vector_, img.descriptors_);

    
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