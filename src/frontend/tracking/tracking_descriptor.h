#pragma once

#include<opencv2/opencv.hpp>
#include<memory>
#include"../../log/logging.h"
#include"../../data/camera.h"

namespace modules_vins{

class BFMatcher{

    public:
    BFMatcher();
    void matching(Image &img0, Image &img1, std::vector<cv::DMatch> &matches);

    protected:

    std::shared_ptr<cv::BFMatcher> bf_;

};
    


} //modules_vins


