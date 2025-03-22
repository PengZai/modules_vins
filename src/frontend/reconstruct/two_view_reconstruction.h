#pragma once

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>


#include "../../system/system_config.h"
#include "../../data/point.h"
#include "../../data/camera.h"
#include "../../log/logging.h"


namespace modules_vins
{


class TwoViewReconstructor{



    public:
    TwoViewReconstructor(const std::shared_ptr<SystemConfig> &sys_config);    
    void reconstruct(Image &img0, Image &img1);

    std::shared_ptr<SystemConfig> sys_config_;

};

    
} // namespace modules_vins








