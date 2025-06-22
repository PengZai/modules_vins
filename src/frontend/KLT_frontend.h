#pragma once

#include "visual_frontend.h"

namespace modules_vins{

class KLTFrontend : public VisualFrontend{


    public:
        KLTFrontend(const std::shared_ptr<SystemConfig> &sys_config);


        void initPipeline(const std::shared_ptr<CameraFrame> &camera_frame) override;
        void normalPipeline(const std::shared_ptr<CameraFrame> &camera_frame) override;


};

} //modules_vins

