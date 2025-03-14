#include "system_config.h"
#include <typeinfo>

namespace modules_vins
{
    
SystemConfig::SystemConfig(){

    this->params_ = std::make_shared<SystemParameters>();
}


void SystemConfig::loadFromPath(const std::string &config_path){

    loadConfigFromPath(config_path);
    std::string node_name;
    node_name = (*this->file_storage_->root().begin()).name();
    this->params_->loadFromNode(std::make_shared<cv::FileNode>((*this->file_storage_)[node_name]));

    std::string relative_folder = this->path_.substr(0, this->path_.find_last_of('/')) + "/";

    // load camera configs and their parameters
    std::string camera_config_path = relative_folder + this->params_->camera_config_name_;
    std::shared_ptr<CameraConfig> camera_config = std::make_shared<CameraConfig>();
    setCameraConfig(camera_config);
    camera_config->loadConfigFromPath(camera_config_path);
    int Nnode = camera_config->file_storage_->root().size();
    if(Nnode < this->params_->max_cameras_){
        VLOG(VERBOSE) << RED << " maxCameras could not be out of configure number of camera" << RESET;
        std::exit(EXIT_FAILURE);
    }

    cv::FileNodeIterator it = camera_config->file_storage_->root().begin();
    for(int i = 0; i < this->params_->max_cameras_; i++, it++){
        
        std::shared_ptr<CameraParameters> camera_params = std::make_shared<CameraParameters>();
        node_name = (*it).name();
        camera_params->loadFromNode(std::make_shared<cv::FileNode>((*camera_config->file_storage_)[node_name]));
        camera_config->params_vector_.emplace_back(camera_params);
        
    }

}


void SystemConfig::setCameraConfig(const std::shared_ptr<CameraConfig> camera_config){
    this->camera_config_ = camera_config;
}

void SystemConfig::loadConfigFromPath(const std::string &config_path){
    
    std::shared_ptr<cv::FileStorage> file_storage = std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
    if (!file_storage->isOpened()) {
        VLOG(KEY) << config_path << " not couldn't be open";
        std::exit(EXIT_FAILURE);
    }
    this->file_storage_ = file_storage;
    this->path_ = config_path;



}

void SystemParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){
    
    
    this->node_ = node;
    parse("log_verbosity", this->log_verbosity_);
    parse("max_cameras", this->max_cameras_);
    parse("max_imus", this->max_imus_);
    
    parse("max_stereo_time_offset", this->max_stereo_time_offset_);
    parse("num_feature_points", this->num_feature_points_);

    

    parse("imu_config_name", this->imu_config_name_);
    parse("camera_config_name", this->camera_config_name_);
    

    
   
    VLOG(VERBOSE) << "config file loaded";

}


} // namespace modules_vins
