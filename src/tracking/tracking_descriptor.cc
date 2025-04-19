#include "tracking_descriptor.h"

namespace modules_vins{



BFMatcher::BFMatcher(){

    this->bf_ = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);

}


void BFMatcher::matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, std::vector<cv::DMatch> &matches){

    this->bf_->match(img0->descriptors_, img1->descriptors_, matches); // Find the two best matches

    // Sort matches based on distance (best matches first)
    std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b) {
        return a.distance < b.distance;
    });

}


} //modules_vins

