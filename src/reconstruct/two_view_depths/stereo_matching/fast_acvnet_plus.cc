#include "fast_acvnet_plus.h"


namespace modules_vins{


FastACVNetPlus::FastACVNetPlus(const std::shared_ptr<SystemConfig> &sys_config, const std::string &model_path):
sys_config_(sys_config),
device_(torch::cuda::is_available() ? torch::kCUDA :torch::kCPU)
{


    this->module_ = torch::jit::load(model_path);
    this->module_.to(this->device_);

}



void FastACVNetPlus::reconstruct(const std::shared_ptr<Image> &left_img, const std::shared_ptr<Image> &right_img){

    
 
    int left_img_height = left_img->color_data_.rows;
    int left_img_width = left_img->color_data_.cols;
    int right_img_height = right_img->color_data_.rows;
    int right_img_width = right_img->color_data_.cols;   

    if(left_img_height != right_img_height || left_img_width != right_img_width){
        
        LOG(INFO) << " left image size " << " left : (" << left_img_height << "," << left_img_width << ") is not equal to right image size : (" <<  right_img_height << "," << right_img_width << ")";
        return;
    }

    // Resize the image
    cv::Mat input_left_img;
    cv::Mat input_right_img;

    cv::cvtColor(left_img->color_data_, input_left_img, cv::COLOR_BGR2RGB);
    cv::cvtColor(right_img->color_data_, input_right_img, cv::COLOR_BGR2RGB);

    
    int left_img_height_i = (left_img_height / 32 + 1) * 32;
    int left_img_width_i = (left_img_width / 32 + 1) * 32;

    int pad_top = left_img_height_i - left_img_height;
    int pad_left = left_img_width_i - left_img_width;

    cv::copyMakeBorder(input_left_img, input_left_img, pad_top, 0, pad_left, 0,
        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::copyMakeBorder(input_right_img, input_right_img, pad_top, 0, pad_left, 0,
        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));


    input_left_img.convertTo(input_left_img, CV_32FC3, 1.0f / 255.0f);
    input_right_img.convertTo(input_right_img, CV_32FC3, 1.0f / 255.0f);


    // // Convert to (normalized) torch Tensor
    torch::Tensor input_left_img_tensor = torch::from_blob(input_left_img.data, {1, input_left_img.rows, input_left_img.cols, 3}, torch::kFloat);
    input_left_img_tensor = input_left_img_tensor.permute({0, 3, 1, 2});

    input_left_img_tensor[0][0] = input_left_img_tensor[0][0].sub_(0.485).div_(0.229);
    input_left_img_tensor[0][1] = input_left_img_tensor[0][1].sub_(0.456).div_(0.224);
    input_left_img_tensor[0][2] = input_left_img_tensor[0][2].sub_(0.406).div_(0.225);
    input_left_img_tensor = input_left_img_tensor.to(this->device_);

    torch::Tensor input_right_img_tensor = torch::from_blob(input_right_img.data, {1, input_right_img.rows, input_right_img.cols, 3});
    input_right_img_tensor = input_right_img_tensor.permute({0, 3, 1, 2});
    input_right_img_tensor[0][0] = input_right_img_tensor[0][0].sub_(0.485).div_(0.229);
    input_right_img_tensor[0][1] = input_right_img_tensor[0][1].sub_(0.456).div_(0.224);
    input_right_img_tensor[0][2] = input_right_img_tensor[0][2].sub_(0.406).div_(0.225);
    input_right_img_tensor = input_right_img_tensor.to(this->device_);

    // torch::Tensor output_tensor = this->module_.forward({ input_left_img_tensor, input_right_img_tensor })
    //                         .toTensor()
    //                         .squeeze()
    //                         .detach()
    //                         .cpu();

    std::vector<torch::Tensor> output_tensor_vector = this->module_.forward({ input_left_img_tensor, input_right_img_tensor })
                            .toTensorVector();
    torch::Tensor output_tensor = output_tensor_vector.back()  // get the last one
                                .squeeze() // (1, 1, H, W) → (H, W)
                                .detach()
                                .cpu(); 

    torch::Tensor cropped_output_disparity = output_tensor.index({
        torch::indexing::Slice(pad_top, torch::indexing::None),                 // height dim (dim 2)
        torch::indexing::Slice(pad_left, torch::indexing::None)                  // width dim (dim 3)
    });
    // cropped_output_tensor = cropped_output_tensor.mul(256).round().to(torch::kUInt16).contiguous();


    Eigen::Matrix<double, 4, 4> T_right_cam_left_cam= this->sys_config_->camera_config_->getExtrinsicsBetweenCamerasBySensorID(right_img->sensor_id_, left_img->sensor_id_);
    Eigen::Vector3d t = T_right_cam_left_cam.block<3,1>(0, 3);  // Get translation vector
    double B = t.norm();
    const Eigen::Matrix3d K_left = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(left_img->sensor_id_)->getIntrinsicsMatrix();
    double fx = K_left(0,0);

    auto valid_mask = cropped_output_disparity > 0;

    torch::Tensor output_depth = torch::zeros_like(cropped_output_disparity);
    output_depth.index_put_({valid_mask}, fx * B / cropped_output_disparity.index({valid_mask}));


    left_img->learned_stereo_depth_ = cv::Mat(left_img_height, left_img_width, CV_32FC1, output_depth.data_ptr<float>()).clone();


    // std::cout <<  left_img->learned_stereo_disparity_ ;

}
    

} //modules_vins