#pragma once


#include <memory>


#include "../system/system_config.h"
#include "../system/state.h"
#include "../data/camera.h"

namespace modules_vins{

class Initializer{

    public:

    Initializer(const std::shared_ptr<SystemConfig> config);

    bool initialize(CameraFrame &camera_frame);

    std::shared_ptr<SystemConfig> sys_config_;


};


} //namespace modules_vins