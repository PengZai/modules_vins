#pragma once
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>


#include "../../../data/camera.h"


namespace modules_vins
{



    
class YOLODetector{

    public:
    YOLODetector(const std::string &model_path);
    float generate_scale(cv::Mat& image, const std::vector<int>& target_size);
    float letterbox(cv::Mat &input_image, cv::Mat &output_image, const std::vector<int> &target_size);
    torch::Tensor xyxy2xywh(const torch::Tensor& x);
    torch::Tensor xywh2xyxy(const torch::Tensor& x);
    torch::Tensor nms(const torch::Tensor& bboxes, const torch::Tensor& scores, float iou_threshold);
    torch::Tensor non_max_suppression(torch::Tensor& prediction, float conf_thres = 0.25, float iou_thres = 0.45, int max_det = 300);
    torch::Tensor clip_boxes(torch::Tensor& boxes, const std::vector<int>& shape);
    torch::Tensor scale_boxes(const std::vector<int>& img1_shape, torch::Tensor& boxes, const std::vector<int>& img0_shape);
    void detect(const std::shared_ptr<Image> &img);


    protected:

    torch::jit::script::Module model_;
    torch::Device device_;
    std::string model_path_;

    std::vector<std::string> classes_;

};

    
} // namespace modules_vins


