#include "camera_config.h"


namespace modules_vins
{



void CameraConfig::calculateExtrinsicsAndProjectionMatrixBetweenCameras(){

    for(int sensor_id_i=0;sensor_id_i<(int)this->params_vector_.size(); sensor_id_i++){

        Eigen::Matrix4d &T_imu_cam_i = this->params_vector_.at(sensor_id_i)->T_imu_cam_;

        for(int sensor_id_j=0;sensor_id_j<(int)this->params_vector_.size(); sensor_id_j++){

            Eigen::Matrix4d &T_imu_cam_j = this->params_vector_.at(sensor_id_j)->T_imu_cam_;
            
            Eigen::Matrix4d T_cam_i_cam_j = T_imu_cam_i.inverse() * T_imu_cam_j;

            this->map_extrinsics_between_cameras_[{sensor_id_i, sensor_id_j}] =  T_cam_i_cam_j;

        }
    }
    
}

 


Eigen::Matrix<double, 4, 4> CameraConfig::getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_i, const unsigned int sensor_id_j){

    return this->map_extrinsics_between_cameras_[{sensor_id_i, sensor_id_j}];

}


CameraParameters::CameraParameters():
    resolution_(2), 
    distortion_coeffs_(4), 
    intrinsics_(4),
    use_sensor_depth_(false),
    use_learned_depth_(false),
    use_stereo_matching_(false),
    use_learned_stereo_matching_(false),
    use_learned_object_detection_(false),
    use_learned_semantic_segmentation_(false)
{}






const Eigen::VectorXd CameraParameters::getDistortionCoeffs(){

    return this->distortion_coeffs_;
}

const cv::Mat CameraParameters::getCVDistortionCoeffs() {

    cv::Mat cv_distortion_coeffs;

    cv::eigen2cv(this->distortion_coeffs_, cv_distortion_coeffs);

    return cv_distortion_coeffs;
}


const Eigen::Matrix3d CameraParameters::getIntrinsicsMatrix(){


    Eigen::Matrix3d K;
    K << this->intrinsics_(0), 0, this->intrinsics_(2),
         0, this->intrinsics_(1), this->intrinsics_(3),
         0, 0, 1;

    return K;
}


const cv::Mat CameraParameters::getCVIntrinsicsMatrix(){
    
    const Eigen::Matrix3d K = getIntrinsicsMatrix();

    cv::Mat cv_K;
    cv::eigen2cv(K, cv_K);

    return cv_K;

}



void CameraParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node)
{

    this->node_ = node;

    parse("rgb_rostopic", this->rgb_rostopic_);

    parse("use_sensor_detph", this->use_sensor_depth_);
    parse("sensor_depth_rostopic", this->sensor_depth_rostopic_);
    
    parse("use_learned_depth", this->use_learned_depth_);
    parse("model_name_learned_depth", this->model_name_learned_depth_);
    parse("output_learned_depth_rostopic", this->output_learned_depth_rostopic_);

    parse("use_stereo_matching", this->use_stereo_matching_);
    parse("stereo_matching_rostopic", this->stereo_matching_rostopic_);

    parse("use_learned_stereo_matching", this->use_learned_stereo_matching_);
    parse("model_name_learned_stereo_matching", this->model_name_learned_stereo_matching_);
    parse("learned_stereo_matching_rostopic", this->learned_stereo_matching_rostopic_);

    parse("use_learned_object_detection", this->use_learned_object_detection_);
    parse("model_name_learned_object_detection", this->model_name_learned_object_detection_);
    parse("learned_object_detection_rostopic", this->learned_object_detection_rostopic_);

    parse("use_learned_semantic_segmentation", this->use_learned_semantic_segmentation_);
    parse("model_name_learned_semantic_segmentation", this->model_name_learned_semantic_segmentation_);
    parse("learned_semantic_segmentation_rostopic", this->learned_semantic_segmentation_rostopic_);


  

    parse("output_rostopic", this->output_rostopic_);
    parse("camera_model", this->camera_model_);
    parse("distortion_model_", this->distortion_model_);

    parse("resolution", this->resolution_);
    parse("distortion_coeffs", this->distortion_coeffs_);
    parse("intrinsics", this->intrinsics_);

    parse("T_imu_cam", this->T_imu_cam_);

}



} // namespace modules_vins