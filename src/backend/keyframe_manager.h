#pragma once

#include <memory> 
#include <cmath> 

#include "../system/system_config.h"
#include "../data/camera.h"
#include "../reconstruct/reconstructor.h"

namespace modules_vins
{

    

class KeyFrameManager{

    public:
    std::shared_ptr<SystemConfig> sys_config_;

    KeyFrameManager(const std::shared_ptr<SystemConfig> &sys_config);
    bool isKeyFrame(const std::shared_ptr<Frame> &frame);
    void updateKeyFrame(const std::shared_ptr<Frame> &frame);

    protected:
    std::vector<std::shared_ptr<Frame>> key_frame_vector_;
    std::shared_ptr<Map> map_;

    std::shared_ptr<Reconstructor> reconstructor_;

};

    
} // namespace modules_vins


