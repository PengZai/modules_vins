#include "feature_points.h"

namespace modules_vins{




ORBFeature::ORBFeature(int num_features){
    this->orb = cv::ORB::create(num_features);
    
}

void ORBFeature::detect(const cv::Mat &img, std::vector<KeyPoint> &keypoints){

    std::vector<cv::KeyPoint> cvKeypoints;
    this->orb->detect(img, cvKeypoints);

    // Convert cv::KeyPoint to modules_vins::KeyPoint
    keypoints.clear();
    keypoints.reserve(cvKeypoints.size());
    for (const cv::KeyPoint& kp : cvKeypoints) {
        keypoints.emplace_back(kp); // Assuming KeyPoint has a constructor accepting cv::KeyPoint
    }
}

void ORBFeature::compute(const cv::Mat &img, std::vector<KeyPoint> &keypoints, cv::Mat &descriptors){

    std::vector<cv::KeyPoint> cvKeypoints;
    this->orb->compute(img, cvKeypoints, descriptors);

    // Convert cv::KeyPoint to modules_vins::KeyPoint
    keypoints.clear();
    keypoints.reserve(cvKeypoints.size());
    for (const cv::KeyPoint& kp : cvKeypoints) {
        keypoints.emplace_back(kp); // Assuming KeyPoint has a constructor accepting cv::KeyPoint
    }

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