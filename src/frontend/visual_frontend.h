#pragma once

#include<vector>
#include<memory>
#include<opencv2/opencv.hpp>

#include "../system/system_config.h"
#include "../data/camera.h"
#include "detect/detector.h"
#include "tracking/tracker.h"

namespace modules_vins{

class VisualFrontend{

    public:

    VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config);

    void pipeline(CameraFrame &camera_frame);

    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::deque<CameraFrame> camera_frame_deque_;
    

    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> trakcer_;

};
    


} // namespace modules_vins