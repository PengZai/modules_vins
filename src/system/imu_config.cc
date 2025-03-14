#include "imu_config.h"


namespace modules_vins
{
    



void ImuConfig::loadConfigFromPath(const std::string &config_path){
    
    std::shared_ptr<cv::FileStorage> config = std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
    if (!config->isOpened()) {
        VLOG(KEY) << config_path << " not couldn't be open";
        std::exit(EXIT_FAILURE);
    }
    VLOG(VERBOSE) << " loading config file, parameters are placed with below";

    // under develping

    VLOG(VERBOSE) << "config file in "  << config_path << " was loaded";

}




} // namespace modules_vins