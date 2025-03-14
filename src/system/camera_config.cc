#include "camera_config.h"


namespace modules_vins
{


CameraParameters::CameraParameters():
    resolution_(2), distortion_coeffs_(4), intrinsics_(4)
{
    
}

void CameraConfig::loadConfigFromPath(const std::string &config_path){
    
    std::shared_ptr<cv::FileStorage> file_storage = std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
    if (!file_storage->isOpened()) {
        VLOG(KEY) << config_path << " not couldn't be open";
        std::exit(EXIT_FAILURE);
    }
    VLOG(VERBOSE) << " loading config file, parameters are placed with below";

    this->file_storage_ = file_storage;
    this->path_ = config_path;


    VLOG(VERBOSE) << "config file in "  << config_path << " was loaded";

}


void CameraParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

    parse("rostopic", this->rostopic_);
    parse("output_rostopic", this->output_rostopic_);
    parse("camera_model", this->camera_model_);
    parse("distortion_model_", this->distortion_model_);

    parse("resolution", this->resolution_);
    parse("distortion_coeffs", this->distortion_coeffs_);
    parse("intrinsics", this->intrinsics_);

    parse("T_imu_cam", this->T_imu_cam_);

}



} // namespace modules_vins