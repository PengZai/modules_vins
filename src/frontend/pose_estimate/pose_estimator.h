#pragma once

#include <opencv2/opencv.hpp>


#include "../../data/camera.h"
#include "../../system/system_config.h"

namespace modules_vins
{




class PoseEstimator{

    public:

    PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config);


    std::shared_ptr<SystemConfig> sys_config_;
    std::deque<CameraFrame> camera_frame_deque_;

   
    void pipeline(CameraFrame &camera_frame);


};


    
} // namespace modules_vins


