#include "tracking_descriptor.h"

namespace modules_vins{



BFMatcher::BFMatcher(){

    this->bf_ = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);

}


void BFMatcher::matching(Image &img0, Image &img1, std::vector<cv::DMatch> &matches){

    this->bf_->match(img0.descriptors_, img1.descriptors_, matches); // Find the two best matches


}


} //modules_vins

