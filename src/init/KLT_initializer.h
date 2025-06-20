#pragma once

#include "initializer.h"



namespace modules_vins{


class KLTInitializer : public Initializer{

    public:
        KLTInitializer(const std::shared_ptr<SystemConfig> &sys_config);

        void pipeline(const std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque) override;



};




}//namespace modules_vins


