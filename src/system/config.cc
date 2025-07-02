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



void Config::calculateExtrinsicsAndProjectionMatrixBetweenSensors(){

    for(size_t sensor_id_i=0;sensor_id_i< this->params_vector_.size(); sensor_id_i++){

        Eigen::Matrix4d &T_imu0_sensor_i = this->params_vector_.at(sensor_id_i)->T_imu0_sensor_;

        for(int sensor_id_j=0;sensor_id_j<(int)this->params_vector_.size(); sensor_id_j++){

            Eigen::Matrix4d &T_imu0_sensor_j = this->params_vector_.at(sensor_id_j)->T_imu0_sensor_;
            
            Eigen::Matrix4d T_sensor_i_sensor_j = T_imu0_sensor_i.inverse() * T_imu0_sensor_j;

            this->sensor_id_sensor_id_extrinsics_map_[{sensor_id_i, sensor_id_j}] =  T_sensor_i_sensor_j;

        }
    }
    
}


Eigen::Matrix<double, 4, 4> Config::getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_i, const unsigned int sensor_id_j){

    return this->sensor_id_sensor_id_extrinsics_map_[{sensor_id_i, sensor_id_j}];

}


} // modules_vins