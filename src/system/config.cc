#include "config.h"


namespace modules_vins{



void Config::loadConfigFromPath(const std::string &config_path){

    std::shared_ptr<cv::FileStorage> file_storage = std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
    if (!file_storage->isOpened()) {
        LOG(INFO) << config_path << " not couldn't be open";
        std::exit(EXIT_FAILURE);
    }
    LOG(INFO) << " loading config file, parameters are placed with below";

    this->file_storage_ = file_storage;
    this->path_ = config_path;


    LOG(INFO) << "config file in "  << config_path << " was loaded";



}

} // modules_vins