#pragma once

#include "visual_frontend.h"

namespace modules_vins{

class DescriptorMatchFrontend : public VisualFrontend{


    public:
        DescriptorMatchFrontend(const std::shared_ptr<SystemConfig> &sys_config);

        void initPipeline(const std::shared_ptr<Frame> &frame) override;

        void normalPipeline(const std::shared_ptr<Frame> &frame) override;


};

} //modules_vins

