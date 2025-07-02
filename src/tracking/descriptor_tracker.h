#pragma once

#include<opencv2/opencv.hpp>
#include<memory>
#include"../log/logging.h"
#include"../data/camera.h"
#include "tracker.h"

namespace modules_vins{

class DescriptorTracker : public Tracker{

    public:
    DescriptorTracker(const std::shared_ptr<SystemConfig> &sys_config);
    void matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, 
        std::vector<cv::DMatch> &good_matches, 
        const float error_threshold = -1.0, const float y_distance_threshold = -1.0);

    void pipeline(const std::shared_ptr<Frame> &frame) override;
    void trackInFrame(const std::shared_ptr<Frame> &frame) override;



    protected:

    std::shared_ptr<cv::BFMatcher> bf_;

};
    


} //modules_vins


