#pragma once

#include <memory> 
#include <cmath> 

#include "../system/system_config.h"
#include "../data/camera.h"

namespace modules_vins
{

    

class KeyFrameManager{

    public:
    std::shared_ptr<SystemConfig> sys_config_;

    KeyFrameManager(const std::shared_ptr<SystemConfig> &sys_config);
    bool isKeyCameraKeyFrame(const std::shared_ptr<CameraFrame> &camera_frame);
    void updateKeyFrame(const std::shared_ptr<CameraFrame> &camera_frame);

    protected:
    std::vector<std::shared_ptr<CameraFrame>> key_camera_frame_vector_;
};

    
} // namespace modules_vins


