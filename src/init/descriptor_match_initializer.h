#pragma once

#include "initializer.h"



namespace modules_vins{


class DescriptorMatchInitializer : public Initializer{

    public:
        DescriptorMatchInitializer(const std::shared_ptr<SystemConfig> &sys_config);

        void pipeline(std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque) override;



};




}//namespace modules_vins


