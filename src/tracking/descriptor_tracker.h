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
    void matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, std::vector<cv::DMatch> &matches);

    void trackInFrame(std::shared_ptr<CameraFrame> &camera_frame) override;
    void trackInTime(std::shared_ptr<Image> &img_from_ref_frame, std::shared_ptr<Image> &img_from_current_frame) override;


    protected:

    std::shared_ptr<cv::BFMatcher> bf_;

};
    


} //modules_vins


