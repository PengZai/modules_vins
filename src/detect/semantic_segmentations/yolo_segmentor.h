#pragma once

#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


#include "../../data/camera.h"

namespace modules_vins{


class YOLOSegmentor{

    public:


    YOLOSegmentor(const std::string &model_path);
    void detect(const std::shared_ptr<Image> &img);
    std::vector<float> Letterbox(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size);
    cv::Rect toBox(const cv::Mat& input, const cv::Rect& range);


    protected:

    const float conf_threshold_ = 0.5;
    const float nms_threshold_ = 0.5;

    torch::jit::script::Module model_;
    torch::Device device_;
    std::string model_path_;

    std::vector<std::string> classes_;


    

};


} //modules_vins