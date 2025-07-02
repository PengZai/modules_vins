#include "config.h"


namespace modules_vins{


void Parameters::setName(const std::string name){
    this->name_ = name;
}


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





Eigen::Matrix<double, 4, 4> Config::getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_i, const unsigned int sensor_id_j){

    return this->sensor_id_sensor_id_extrinsics_map_[{sensor_id_i, sensor_id_j}];

}


} // modules_vins