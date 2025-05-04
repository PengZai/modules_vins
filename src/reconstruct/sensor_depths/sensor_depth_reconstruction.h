#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


#include "../../system/system_config.h"
#include "../../data/point.h"
#include "../../data/camera.h"
#include "../../log/logging.h"


namespace modules_vins
{


class SensorDepthReconstruction{



    public:
    SensorDepthReconstruction(const std::shared_ptr<SystemConfig> &sys_config);    
    void reconstruct(const std::shared_ptr<Image> &img);

    std::shared_ptr<SystemConfig> sys_config_;

};

    
} // namespace modules_vins








