#include "foundation_stereo.h"


namespace modules_vins{


FoundationStereo::FoundationStereo(const std::shared_ptr<SystemConfig> &sys_config, const std::string &model_path):
sys_config_(sys_config),
device_(torch::cuda::is_available() ? torch::kCUDA :torch::kCPU)
{


    this->module_ = torch::jit::load(model_path);
    this->module_.to(this->device_);
    this->module_.eval();

}



void FoundationStereo::reconstruct(const std::shared_ptr<Image> &left_img, const std::shared_ptr<Image> &right_img){

    
    // at::cuda::CUDACachingAllocator::emptyCache();

    // Resize the image
    cv::Mat input_left_img;
    cv::Mat input_right_img;

    int input_width = 960;
    int input_height = 600;

    cv::resize(left_img->color_data_, input_left_img, cv::Size(input_width, input_height));
    cv::resize(right_img->color_data_, input_right_img, cv::Size(input_width, input_height));
 
    int left_img_height = input_left_img.rows;
    int left_img_width = input_left_img.cols;
    int right_img_height = input_right_img.rows;
    int right_img_width = input_right_img.cols;   

    if(left_img_height != right_img_height || left_img_width != right_img_width){
        
        LOG(INFO) << " left image size " << " left : (" << left_img_height << "," << left_img_width << ") is not equal to right image size : (" <<  right_img_height << "," << right_img_width << ")";
        return;
    }


    cv::cvtColor(input_left_img, input_left_img, cv::COLOR_BGR2RGB);
    cv::cvtColor(input_right_img, input_right_img, cv::COLOR_BGR2RGB);

    int divis_by = 32;
    
    int pad_ht = ((left_img_height / divis_by + 1) * divis_by - left_img_height) % divis_by;
    int pad_wd = ((left_img_width / divis_by + 1) * divis_by - left_img_width) % divis_by;

    int pad_top = pad_ht / 2;
    int pad_bottom = pad_ht - pad_ht / 2;
    int pad_left = pad_wd / 2;
    int pad_right = pad_wd - pad_wd / 2;


    cv::copyMakeBorder(input_left_img, input_left_img, pad_top, pad_bottom, pad_left, pad_right,
        cv::BORDER_REPLICATE);
    cv::copyMakeBorder(input_right_img, input_right_img, pad_top, pad_bottom, pad_left, pad_right,
        cv::BORDER_REPLICATE);


    input_left_img.convertTo(input_left_img, CV_32FC3, 1.0f / 255.0f);
    input_right_img.convertTo(input_right_img, CV_32FC3, 1.0f / 255.0f);


    // // Convert to (normalized) torch Tensor
    torch::Tensor input_left_img_tensor = torch::from_blob(input_left_img.data, {1, input_left_img.rows, input_left_img.cols, 3}, torch::kFloat);
    input_left_img_tensor = input_left_img_tensor.permute({0, 3, 1, 2});
    // input_left_img_tensor[0][0] = input_left_img_tensor[0][0].sub_(0.485).div_(0.229);
    // input_left_img_tensor[0][1] = input_left_img_tensor[0][1].sub_(0.456).div_(0.224);
    // input_left_img_tensor[0][2] = input_left_img_tensor[0][2].sub_(0.406).div_(0.225);
    input_left_img_tensor = input_left_img_tensor.to(this->device_);

    torch::Tensor input_right_img_tensor = torch::from_blob(input_right_img.data, {1, input_right_img.rows, input_right_img.cols, 3});
    input_right_img_tensor = input_right_img_tensor.permute({0, 3, 1, 2});
    // input_right_img_tensor[0][0] = input_right_img_tensor[0][0].sub_(0.485).div_(0.229);
    // input_right_img_tensor[0][1] = input_right_img_tensor[0][1].sub_(0.456).div_(0.224);
    // input_right_img_tensor[0][2] = input_right_img_tensor[0][2].sub_(0.406).div_(0.225);
    input_right_img_tensor = input_right_img_tensor.to(this->device_);
    
    torch::NoGradGuard no_grad;  // disables gradient tracking
    torch::Tensor output_tensor = this->module_.forward({ input_left_img_tensor, input_right_img_tensor })
                            .toTensor()
                            .squeeze()
                            .detach()
                            .cpu();


    // torch::Tensor cropped_output_disparity = output_tensor.index({
    //     torch::indexing::Slice(pad_top, torch::indexing::None),                 // height dim (dim 2)
    //     torch::indexing::Slice(pad_left, torch::indexing::None)                  // width dim (dim 3)
    // });
    // cropped_output_tensor = cropped_output_tensor.mul(256).round().to(torch::kUInt16).contiguous();


    // Eigen::Matrix<double, 4, 4> T_right_cam_left_cam= this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(right_img->sensor_id_, left_img->sensor_id_);
    // Eigen::Vector3d t = T_right_cam_left_cam.block<3,1>(0, 3);  // Get translation vector
    // double B = t.norm();
    // const Eigen::Matrix3d K_left = this->sys_config_->camera_config_->params_vector_.at(left_img->sensor_id_)->getIntrinsicsMatrix();
    // double fx = K_left(0,0);

    // auto valid_mask = cropped_output_disparity > 0;

    // torch::Tensor output_depth = torch::zeros_like(cropped_output_disparity);
    // output_depth.index_put_({valid_mask}, fx * B / cropped_output_disparity.index({valid_mask}));
    double min_val, max_val;
    int output_height = output_tensor.size(0);
    int output_width = output_tensor.size(1);
    left_img->learned_stereo_depth_ = cv::Mat(output_height, output_width, CV_32FC1, output_tensor.data_ptr<float>()).clone();
    LOG(INFO) << left_img->learned_stereo_depth_;
    cv::Mat depth_visual = left_img->learned_stereo_depth_.clone();
    cv::minMaxLoc(depth_visual, &min_val, &max_val);
    depth_visual = 255 * (depth_visual - min_val) / (max_val - min_val);
    depth_visual.convertTo(depth_visual, CV_8U);
    cv::applyColorMap(depth_visual, depth_visual, cv::COLORMAP_JET); //COLORMAP_HOT, COLORMAP_JET

    cv::imshow("learned stereo disparity in frame for x " + std::to_string(0), depth_visual);
    cv::waitKey(0);
    
    cv::resize(left_img->learned_stereo_depth_, left_img->learned_stereo_depth_, cv::Size(input_width, input_height));


    // std::cout <<  left_img->learned_stereo_disparity_ ;

}
    

} //modules_vins