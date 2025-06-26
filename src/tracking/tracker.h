#pragma once


#include<memory>

#include "../detect/detector.h"
#include "../data/camera.h"
#include "../system/system_config.h"
#include "../reconstruct/two_view_depths/two_view_reconstruction.h"
#include "../utils/utils.h"

namespace modules_vins{

class Tracker{

    public:

    Tracker(const std::shared_ptr<SystemConfig> &sys_config);
    virtual ~Tracker() = default;

    virtual void trackInFrame(const std::shared_ptr<CameraFrame> &camera_frame) = 0;
    virtual void pipeline(const std::shared_ptr<CameraFrame> &camera_frame) = 0;


    protected:

    std::shared_ptr<SystemConfig> sys_config_;

};


} //modules_vins