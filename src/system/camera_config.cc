#include "camera_config.h"


namespace modules_vins
{




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

void CameraConfig::calculateExtrinsicsAndProjectionMatrixBetweenCameras(){

    for(int sensor_id_i=0;sensor_id_i<(int)this->params_vector_.size(); sensor_id_i++){

        Eigen::Matrix4d &T_imu_cam_i = this->params_vector_.at(sensor_id_i)->T_imu_cam_;

        for(int sensor_id_j=0;sensor_id_j<(int)this->params_vector_.size(); sensor_id_j++){

            std::shared_ptr<CameraParameters> sensor_j_params_ptr = this->params_vector_.at(sensor_id_j);
            Eigen::Matrix4d &T_imu_cam_j = sensor_j_params_ptr->T_imu_cam_;
            
            Eigen::Matrix4d T_cam_i_cam_j = T_imu_cam_i.inverse() * T_imu_cam_j;
            Eigen::Matrix3d K_j = sensor_j_params_ptr->getIntrinsicsMatrix();

            this->map_extrinsics_between_cameras_[{sensor_id_i, sensor_id_j}] =  T_cam_i_cam_j;

        }
    }
    
}

 


Eigen::Matrix4d CameraConfig::getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_i, const unsigned int sensor_id_j){

    return this->map_extrinsics_between_cameras_[{sensor_id_i, sensor_id_j}];

}
Eigen::Matrix<double, 3, 4> CameraConfig::getProjectionMatrixBetweenCamerasBySensorID(Eigen::Matrix4d T_cam_i_world, const unsigned int sensor_id_i, const unsigned int sensor_id_j){

    std::shared_ptr<CameraParameters> sensor_j_params_ptr = this->params_vector_.at(sensor_id_j);

    Eigen::Matrix3d K_j = sensor_j_params_ptr->getIntrinsicsMatrix();

    Eigen::Matrix4d T_cam_j_cam_i = getExtrinsicsBetweenCamerasBySensorID(sensor_id_j, sensor_id_i);

    Eigen::Matrix<double, 3, 4> projection_matrix_cam_j_world = K_j * (T_cam_j_cam_i *T_cam_i_world).topRows(3);

    return projection_matrix_cam_j_world;
}

CameraParameters::CameraParameters():
    resolution_(2), distortion_coeffs_(4), intrinsics_(4)
{}

Eigen::Matrix3d CameraParameters::getIntrinsicsMatrix(){


    Eigen::Matrix3d K;
    K << this->intrinsics_(0), 0, this->intrinsics_(2),
         0, this->intrinsics_(1), this->intrinsics_(3),
         0, 0, 1;

    return K;
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