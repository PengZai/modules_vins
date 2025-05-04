#pragma once



#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

#include "../../../system/system_config.h"
#include "../../../data/camera.h"


namespace modules_vins{


class FastACVNetPlus{

    public:

    
    FastACVNetPlus(const std::shared_ptr<SystemConfig> &sys_config, const std::string &model_path);
    void reconstruct(const std::shared_ptr<Image> &left_img, const std::shared_ptr<Image> &right_img);

    protected:

    // Network input size (predefined)

    std::shared_ptr<SystemConfig> sys_config_;
    std::string model_path_;
    torch::jit::script::Module module_;            /// Torch model
    torch::Device device_;

};





} //modules_vins


