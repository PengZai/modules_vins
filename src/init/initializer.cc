#include "initializer.h"


namespace modules_vins{

Initializer::Initializer(const std::shared_ptr<SystemConfig> config):
sys_config_(config)
{

}



    
bool Initializer::initialize(CameraFrame &camera_frame, State &state){

    std::shared_ptr<Image> img_0;
    for(int i=0; i < (int)camera_frame.image_vector_.size(); i++){
        img_0 = camera_frame.image_vector_.at(i);
        std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);
        img_i->initPose();
    }

    if(!img_0){
        return false;
    }

    Eigen::Vector3d coordinate_t(0,0,0);  
    Eigen::Matrix<double, 3, 3> coordinate_R = this->sys_config_->params_->T_cam_GT_.block<3,3>(0,0);
    double coordinate_R_det = coordinate_R.determinant();
    if(std::abs(1-coordinate_R_det) > 0.1){
        VLOG(VERBOSE) <<  RED << "det of extrinsic between GT and CAM small than 1 too much" << RESET;
        return false;
    }

    Sophus::SE3<double> T_cam_GT = Sophus::SE3<double>(Sophus::SO3d::fitToSO3<double>(coordinate_R), coordinate_t);

    state.SynchronizeAndTransformGTPoseWithTcw(img_0->timestamp_, this->sys_config_->params_->max_tolerant_gt_time_offset_, T_cam_GT);

    return true;
}
    
    
} //namespace modules_vins