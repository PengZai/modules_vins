#include "config.h"


namespace modules_vins{



void Config::loadConfigFromPath(const std::string &config_path){

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

} // modules_vins