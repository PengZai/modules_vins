#pragma once



#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

#include "../../../data/camera.h"


namespace modules_vins{


class MiDas{

    public:

    
    MiDas(const std::string &model_path);
    void reconstruct(const std::shared_ptr<Image> &img);

    protected:

    // Network input size (predefined)
    const int input_width_ = 384;
    const int input_height_ = 288;
    
    std::string model_path_;
    torch::jit::script::Module module_;            /// Torch model
    torch::Device device_;

};





} //modules_vins


