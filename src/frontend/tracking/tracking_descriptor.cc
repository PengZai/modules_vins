#include "tracking_descriptor.h"

namespace modules_vins{



BFMatcher::BFMatcher(){

    this->bf_ = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);

}


void BFMatcher::matching(cv::Mat &descriptors1, cv::Mat &descriptors2){

    std::vector<cv::DMatch> matches;
    this->bf_->match(descriptors1, descriptors2, matches); // Find the two best matches
    // for(const cv::DMatch & match: matches){
    //     VLOG(VERBOSE) << match.imgIdx;
    // }


}


} //modules_vins

