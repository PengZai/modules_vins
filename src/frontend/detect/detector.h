#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>

#include "../../data/camera.h"
#include "../../system/system_config.h"
#include "features/feature_points.h"


namespace modules_vins
{

class Detector{

    public:
    Detector(const std::shared_ptr<SystemConfig> &sys_config);
    void pipeline(CameraFrame &camera_frame);
    void detect(Image &img);
    void computeDescriptor(Image &img);


    protected:
    std::shared_ptr<ORBFeature> orb_feature_;
    std::shared_ptr<SystemConfig> sys_config_;

};

    
} // namespace modules_vins

