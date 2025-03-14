#include "feature_points.h"

namespace modules_vins{




ORBFeature::ORBFeature(int num_features){
    this->orb = cv::ORB::create(num_features);
    
}

void ORBFeature::detect(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints){

    this->orb->detect(img, keypoints);
}

void ORBFeature::compute(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors){

    this->orb->compute(img, keypoints, descriptors);

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