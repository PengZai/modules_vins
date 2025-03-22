#pragma once

#include <memory>

#include "two_view_reconstruction.h"
#include "../../data/camera.h"
#include "../../system/system_config.h"
#include "../../data/camera.h"



namespace modules_vins
{


class Reconstructor{

    public:

    Reconstructor(const std::shared_ptr<SystemConfig> &sys_config);
    void pipeline(CameraFrame &camera_frame);

    std::shared_ptr<SystemConfig> sys_config_;


    protected:
    std::shared_ptr<TwoViewReconstructor> two_view_reconstructor_;

};
    
} // namespace modules_vins


