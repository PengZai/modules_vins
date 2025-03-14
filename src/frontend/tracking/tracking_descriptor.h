#pragma once

#include<opencv2/opencv.hpp>
#include<memory>
#include"../../log/logging.h"

namespace modules_vins{

class BFMatcher{

    public:
    BFMatcher();
    void matching(cv::Mat &descriptors1, cv::Mat &descriptors2);

    protected:

    std::shared_ptr<cv::BFMatcher> bf_;

};
    


} //modules_vins


