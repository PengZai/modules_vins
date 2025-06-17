#include "initializer.h"


namespace modules_vins{

Initializer::Initializer(const std::shared_ptr<SystemConfig> sys_config):
sys_config_(sys_config),
origin_index_in_camera_frame_deque_(0),
num_fail_(0),
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


    this->detector_ = std::make_shared<Detector>(sys_config);
    this->trakcer_ = std::make_shared<Tracker>(sys_config);
    this->reconstructor_ = std::make_shared<Reconstructor>(sys_config);
    this->pose_estimator_ = std::make_shared<PoseEstimator>(sys_config);

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




const std::deque<std::shared_ptr<CameraFrame>> &Initializer::getInitializedReferenceCameraFrameDeque() const{

    return this->ref_camera_frame_deque_;
}


bool Initializer::checkSuccess(){
    

    double culmulative_trans = getCumulativeTranslationInCameraFrameDeque(this->ref_camera_frame_deque_);

    LOG(INFO) << " cumulative value of translation in reference camera frame deque is : " << culmulative_trans;

    if(culmulative_trans>=this->sys_config_->params_->minimum_cumulative_translation_ &&
         this->ref_camera_frame_deque_.size() >= this->sys_config_->params_->minimum_num_in_ref_camera_frame_ &&
          this->camera_frame_deque_.size() - (index_in_camera_frame_deque_for_latest_ref_camera_frame_ + 1) <= this->sys_config_->params_->maximum_num_fail_ ){
        return true;
    }

    return false;
}

double Initializer::getCumulativeTranslationInCameraFrameDeque(const std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque){
    
    double culmulative_trans = 0;
    if(camera_frame_deque.size() > 0){

        double prev_Tcw_translation_norm = camera_frame_deque.back()->image_vector_.at(0)->T_c_w_.translation().norm();
        for(int i=camera_frame_deque.size()-2; i >= 0; i--){

            const std::shared_ptr<CameraFrame> &camera_frame = camera_frame_deque.at(i);

            const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);
            double Tcw_translation_norm = img_0->T_c_w_.translation().norm();
            culmulative_trans  += Tcw_translation_norm - prev_Tcw_translation_norm;
            prev_Tcw_translation_norm = Tcw_translation_norm;
        }

    }
    

    return culmulative_trans;
}


void Initializer::updateStatus(std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque){


    std::shared_ptr<CameraFrame> &latest_camera_frame = this->camera_frame_deque_.back();


    if(this->status_ == Initializer::Status::NOT_INITIALIZED_YET){
        this->camera_frame_deque_.clear();
        this->ref_camera_frame_deque_.clear();

        this->camera_frame_deque_.push_back(latest_camera_frame);
        this->ref_camera_frame_deque_.push_front(latest_camera_frame);
        
        this->origin_index_in_camera_frame_deque_ = 0;
        this->camera_frame_index_ = this->origin_index_in_camera_frame_deque_+1;
        this->index_in_camera_frame_deque_for_latest_ref_camera_frame_ = this->origin_index_in_camera_frame_deque_;
        LOG(INFO) << GREEN << "reset nitializer::Status from NOT_INITIALIZED_YET to EXTEND"  << RESET;
        this->status_ = Initializer::Status::EXTEND;
        return;
    }


    if(this->status_ == Initializer::Status::EXTEND){
        this->camera_frame_deque_.push_back(latest_camera_frame);
        this->status_ = Initializer::Status::WORKING;
    }


    while(this->status_ == Initializer::Status::WORKING){

        std::shared_ptr<CameraFrame> &current_camera_frame = this->camera_frame_deque_.at(camera_frame_index_);
        current_camera_frame->status_ = CameraFrame::Status::NORMAL;

        for(int i=0; i<this->ref_camera_frame_deque_.size();i++){

            std::shared_ptr<CameraFrame> &ref_camera_frame = ref_camera_frame_deque_.at(i);
            current_camera_frame->ref_camera_frame_ = ref_camera_frame;
            LOG(INFO) << "estimating pose betweeen reference frame with id: " << ref_camera_frame->id_ << " and current frame with id: " << current_camera_frame->id_;

            this->trakcer_->pipeline(current_camera_frame);
            this->pose_estimator_->pipeline(current_camera_frame);
    
            // std::shared_ptr<Image> img_0_from_current_frame = current_camera_frame->image_vector_.at(0);
            // std::shared_ptr<Image> img_0_from_ref_frame = ref_camera_frame->image_vector_.at(0);
            // double diff_trans = std::abs(img_0_from_current_frame->T_c_w_.translation().norm() - img_0_from_ref_frame->T_c_w_.translation().norm());
            // LOG(INFO) << "estimated difference of translation: " << diff_trans;

            if(current_camera_frame->status_ == CameraFrame::Status::NORMAL){
                break;
            }
    
        }
    
        if(current_camera_frame->status_ == CameraFrame::Status::NORMAL){
            LOG(INFO) << GREEN << " correct pose estimation, add camera frame id : " <<  this->camera_frame_deque_.at(this->camera_frame_index_)->id_  << " to ref_camera_frame_deque_"<< RESET;
            this->ref_camera_frame_deque_.push_front(current_camera_frame);
            index_in_camera_frame_deque_for_latest_ref_camera_frame_ = this->camera_frame_index_;
            if(checkSuccess()){
                LOG(INFO) << GREEN << "initialization process successes, return"  << RESET;
                this->status_ = Initializer::Status::SUCCESS;                
                return;
            }
 
        }
        else{
            if( camera_frame_index_ - index_in_camera_frame_deque_for_latest_ref_camera_frame_ > this->sys_config_->params_->maximum_num_fail_){
                LOG(INFO) << RED <<"fail to initialize system with camera frame id: " << this->camera_frame_deque_.at(this->origin_index_in_camera_frame_deque_)->id_  << RESET;
                LOG(INFO) << GREEN << "reset nitializer::Status from WORKING to FAIL"  << RESET;
                this->status_ = Initializer::Status::FAIL; 

            }
            else{
                LOG(INFO) << YELLOW << "wrong pose estimation, skip the camera frame id : " <<  this->camera_frame_deque_.at(this->camera_frame_index_)->id_ << RESET;
            }
        }


        printfStatus();

        if(this->status_ == Initializer::Status::WORKING){
            this->camera_frame_index_++;
            if(this->camera_frame_index_ >= this->camera_frame_deque_.size()){
                LOG(INFO) << YELLOW << "not enough camera frames for initialization, we have to extend camera frame deque"  << RESET;
                this->status_ = Initializer::Status::EXTEND;  
            }
        }


        if(this->status_ == Initializer::Status::FAIL){
            this->ref_camera_frame_deque_.clear();
            for(int i=0;i<camera_frame_deque_.size();i++){
                this->camera_frame_deque_.at(i)->cleanTrackInTimeRelationship();
            }

            this->origin_index_in_camera_frame_deque_++;
            this->index_in_camera_frame_deque_for_latest_ref_camera_frame_ = this->origin_index_in_camera_frame_deque_;
            this->camera_frame_index_ = this->origin_index_in_camera_frame_deque_+1;
            this->ref_camera_frame_deque_.push_front(this->camera_frame_deque_.at(this->origin_index_in_camera_frame_deque_));
            LOG(INFO) << RED << "we reset origin of system with camera frame id: " << this->camera_frame_deque_.at(this->origin_index_in_camera_frame_deque_)->id_  << RESET;
            LOG(INFO) << GREEN << "reset nitializer::Status from FAIL to WORKING"  << RESET;
            this->status_ = Initializer::Status::WORKING;
        }

    }

    

}


void Initializer::printfStatus(){

    LOG(INFO) << GREEN <<"Initializer::Status: " << StatusToString(this->status_) << RESET;
    LOG(INFO) << GREEN << "camera_frame_deque_.size : " << this->camera_frame_deque_.size() << RESET;
    LOG(INFO) << GREEN << "origin_index_in_camera_frame_deque_ : " <<  this->origin_index_in_camera_frame_deque_ << RESET;
    LOG(INFO) << GREEN << "camera_frame_index_ : " <<  this->camera_frame_index_ << RESET;
    LOG(INFO) << GREEN << "ref_camera_frame_deque_.size : " << this->ref_camera_frame_deque_.size() << RESET;
    LOG(INFO) << GREEN << "index_in_camera_frame_deque_for_latest_ref_camera_frame_ : " <<  this->index_in_camera_frame_deque_for_latest_ref_camera_frame_ << RESET;
    LOG(INFO) << GREEN << "culmulative translation in reference cmaera frame deque : " <<  getCumulativeTranslationInCameraFrameDeque(this->ref_camera_frame_deque_) << RESET;

}






void Initializer::pipeline(std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque){



    std::shared_ptr<CameraFrame> &camera_frame = this->camera_frame_deque_.back();

    LOG(INFO) << "Initializer start with camera frame id: " << camera_frame->id_;


    this->detector_->pipeline(camera_frame);


    updateStatus(camera_frame_deque);


    this->reconstructor_->pipeline(camera_frame);

    
    
    LOG(INFO) << "Initializer end with camera frame id: " << camera_frame->id_;
    


}

    
    
} //namespace modules_vins