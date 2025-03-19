#pragma once


#include<memory>

#include "../../data/camera.h"
#include "../../system/system_config.h"
#include "tracking_descriptor.h"


namespace modules_vins{

class Tracker{

    public:

    Tracker(const std::shared_ptr<SystemConfig> &sys_config);
    void trackInFrame(Image &img0, Image &img1);
    void trackInTime(Image &img0, Image &img1);
    void pipeline(CameraFrame &camera_frame);

    protected:

    std::shared_ptr<SystemConfig> sys_config_;
    std::deque<CameraFrame> camera_frame_deque_;
    std::shared_ptr<BFMatcher> bf_;

};


} //modules_vins