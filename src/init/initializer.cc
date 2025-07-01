#include "initializer.h"


namespace modules_vins{

Initializer::Initializer(const std::shared_ptr<SystemConfig> sys_config):
sys_config_(sys_config)
{

    // get extrinsic between camera and GT coordinate
    // Eigen::Vector3d coordinate_t(0,0,0);  
    // Eigen::Matrix<double, 3, 3> coordinate_R = this->sys_config_->params_->T_cam_GT_.block<3,3>(0,0);
    // double coordinate_R_det = coordinate_R.determinant();
    // if(std::abs(1-coordinate_R_det) > 0.1){
    //     LOG(INFO) <<  RED << "det of extrinsic between GT and CAM small than 1 too much" << RESET;
    //     std::exit(EXIT_FAILURE);
    // }

    // this->T_cam_GT_ = Sophus::SE3<double>(Sophus::SO3d::fitToSO3<double>(coordinate_R), coordinate_t);

    
}


    
bool Initializer::initializeGTTcwWithCameraFrame(const std::shared_ptr<CameraFrame> &camera_frame, const std::shared_ptr<State> &state){

    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    if(!img_0){
        return false;
    }

    state->SynchronizeAndTransformComparisonPoseToTcw(img_0->timestamp_);

    return true;
}





} //namespace modules_vins


