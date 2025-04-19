#include "MiDas_reconstruction.h"


namespace modules_vins{


MiDas::MiDas(const std::string &model_path):
device_(torch::cuda::is_available() ? torch::kCUDA :torch::kCPU)
{


    module_ = torch::jit::load(model_path);
    module_.to(this->device_);

}



void MiDas::reconstruct(const std::shared_ptr<Image> &img){

    

    // Resize the image
    cv::Mat input_cv;

    cv::resize(img->color_data_, input_cv, cv::Size(input_width_, input_height_));

    cv::cvtColor(input_cv, input_cv, cv::COLOR_BGR2RGB);
    input_cv.convertTo(input_cv, CV_32FC3, 1.0f / 255.0f);

    // Convert to (normalized) torch Tensor
    torch::Tensor input_tensor = torch::from_blob(input_cv.data, {1, input_height_, input_width_, 3});
    input_tensor = input_tensor.permute({0, 3, 1, 2});
    input_tensor[0][0] = input_tensor[0][0].sub_(0.485).div_(0.229);
    input_tensor[0][1] = input_tensor[0][1].sub_(0.456).div_(0.224);
    input_tensor[0][2] = input_tensor[0][2].sub_(0.406).div_(0.225);
    input_tensor = input_tensor.to(this->device_);
    // Run network inference
    torch::Tensor output_tensor = module_.forward({input_tensor})
                            .toTensor()
                            .squeeze()
                            .detach()
                            .cpu();

    cv::Mat output_mat = cv::Mat(input_height_, input_width_, CV_32FC1, output_tensor.data_ptr<float>());
    // Resize the depth map
    cv::resize(output_mat, img->learned_depth_, cv::Size(img->color_data_.cols, img->color_data_.rows));
    
    VLOG(VERBOSE) << "just test";

}
    

} //modules_vins