#pragma once


#include <memory>


#include "../system/system_config.h"
#include "../system/state.h"
#include "../data/camera.h"

namespace modules_vins{

class Initializer{

    public:

    Initializer(const std::shared_ptr<SystemConfig> config);

    bool initializeGTTcwWithCameraFrame(std::shared_ptr<CameraFrame> &camera_frame, State &state);

    std::shared_ptr<SystemConfig> sys_config_;

    protected:
    Sophus::SE3<double> T_cam_GT_;


};


} //namespace modules_vins