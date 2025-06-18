#include "initializer.h"


namespace modules_vins{

Initializer::Initializer(const std::shared_ptr<SystemConfig> sys_config):
sys_config_(sys_config),
status_(Initializer::Status::NOT_INITIALIZED_YET)
{

    // get extrinsic between camera and GT coordinate
    Eigen::Vector3d coordinate_t(0,0,0);  
    Eigen::Matrix<double, 3, 3> coordinate_R = this->sys_config_->params_->T_cam_GT_.block<3,3>(0,0);
    double coordinate_R_det = coordinate_R.determinant();
    if(std::abs(1-coordinate_R_det) > 0.1){
        LOG(INFO) <<  RED << "det of extrinsic between GT and CAM small than 1 too much" << RESET;
        std::exit(EXIT_FAILURE);
    }

    this->T_cam_GT_ = Sophus::SE3<double>(Sophus::SO3d::fitToSO3<double>(coordinate_R), coordinate_t);


}


void Initializer::setDetector(const std::shared_ptr<Detector> &detector){

    this->detector_ = detector;
}

void Initializer::setTracker(const std::shared_ptr<Tracker> &tracker){

    this->tracker_ = tracker;
}

void Initializer::setReconstructor(const std::shared_ptr<Reconstructor> &reconstructor){

    this->reconstructor_ = reconstructor;
}

void Initializer::setPoseEstimator(const std::shared_ptr<PoseEstimator> &pose_estimator){

    this->pose_estimator_ = pose_estimator;
}



Initializer::Status Initializer::getStatus(){

    return this->status_;
}



    
bool Initializer::initializeGTTcwWithCameraFrame(const std::shared_ptr<CameraFrame> &camera_frame, State &state){

    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    if(!img_0){
        return false;
    }

    state.SynchronizeAndTransformGTPoseToTcw(img_0->timestamp_, this->sys_config_->params_->max_tolerant_gt_time_offset_, this->T_cam_GT_);

    return true;
}




const std::deque<std::shared_ptr<CameraFrame>> &Initializer::getGoodInitializedReferenceCameraFrameDeque() const{

    return this->good_ref_camera_frame_deque_;
}





void Initializer::updateStatus(const std::shared_ptr<CameraFrame> &latest_camera_frame){

    double Tcw_translation_norm = latest_camera_frame->image_vector_.at(0)->T_c_w_.translation().norm();

    if(latest_camera_frame->status_ == CameraFrame::NORMAL){

            this->status_ = Status::SUCCESS;
            this->good_ref_camera_frame_deque_.push_back(latest_camera_frame);
            this->good_ref_camera_frame_deque_.push_back(latest_camera_frame->ref_camera_frame_);

            this->ref_camera_frame_deque_.clear();
    }
    else{
            latest_camera_frame->cleanTrackInTimeRelationship();
    }
        
    
}


void Initializer::printfStatus(){

    LOG(INFO) << GREEN << "Initializer::Status: " << StatusToString(this->status_) << RESET;
    LOG(INFO) << GREEN << "ref_camera_frame_deque_.size : " << this->ref_camera_frame_deque_.size() << RESET;

}



} //namespace modules_vins


