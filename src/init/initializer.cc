#include "initializer.h"


namespace modules_vins{

Initializer::Initializer(const std::shared_ptr<SystemConfig> sys_config):
sys_config_(sys_config),
origin_index_(0),
ref_camera_frame_index_(0),
num_fail_(0),
max_num_fail_(2),
max_num_ref_camera_frame_(3),
status_(Initializer::Status::NOT_INITIALIZED_YET)
{

    // get extrinsic between camera and GT coordinate
    Eigen::Vector3d coordinate_t(0,0,0);  
    Eigen::Matrix<double, 3, 3> coordinate_R = this->sys_config_->params_->T_cam_GT_.block<3,3>(0,0);
    double coordinate_R_det = coordinate_R.determinant();
    if(std::abs(1-coordinate_R_det) > 0.1){
        VLOG(VERBOSE) <<  RED << "det of extrinsic between GT and CAM small than 1 too much" << RESET;
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



    
bool Initializer::initializeGTTcwWithCameraFrame(std::shared_ptr<CameraFrame> &camera_frame, State &state){

     std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    if(!img_0){
        return false;
    }

    state.SynchronizeAndTransformGTPoseToTcw(img_0->timestamp_, this->sys_config_->params_->max_tolerant_gt_time_offset_, this->T_cam_GT_);

    return true;
}

void Initializer::updateStatus(std::shared_ptr<CameraFrame> &latest_camera_frame){


    this->camera_frame_deque_.push_back(latest_camera_frame);

    if(this->status_ == Initializer::Status::NOT_INITIALIZED_YET){
        this->ref_camera_frame_deque_.push_front(this->camera_frame_deque_.at(this->origin_index_));
        this->status_ = Initializer::Status::WORKING;
        return;
    }

    if(this->status_ == Initializer::Status::EXTEND){
        this->status_ = Initializer::Status::WORKING;
    }


    int camera_frame_index = this->origin_index_+1;
    int ref_camera_frame_latest_index = this->origin_index_;

    while(this->status_ == Initializer::Status::WORKING || this->status_ == Initializer::Status::FAIL){

        std::shared_ptr<CameraFrame> &current_camera_frame = this->camera_frame_deque_.at(camera_frame_index);
        current_camera_frame->status_ = CameraFrame::Status::NORMAL;

        if(this->status_ == Initializer::Status::FAIL){
            ref_camera_frame_deque_.clear();
            this->origin_index_++;
            ref_camera_frame_latest_index = this->origin_index_;
            camera_frame_index = this->origin_index_+1;
            this->ref_camera_frame_deque_.push_front(this->camera_frame_deque_.at(this->origin_index_));
            this->status_ = Initializer::Status::WORKING;
            continue;
        }

        for(int i=0; i<this->ref_camera_frame_deque_.size();i++){

            std::shared_ptr<CameraFrame> &ref_camera_frame = ref_camera_frame_deque_.at(i);
    
            this->trakcer_->pipeline(ref_camera_frame, current_camera_frame);
            this->pose_estimator_->pipeline(ref_camera_frame, current_camera_frame);
    
            std::shared_ptr<Image> img_0_from_current_frame = current_camera_frame->image_vector_.at(0);
            std::shared_ptr<Image> img_0_from_ref_frame = ref_camera_frame->image_vector_.at(0);
            double diff_trans = std::abs(img_0_from_current_frame->T_c_w_.translation().norm() - img_0_from_ref_frame->T_c_w_.translation().norm());
    
    
            if(current_camera_frame->status_ == CameraFrame::Status::NORMAL){
                break;
            }
    
        }
    
        if(current_camera_frame->status_ == CameraFrame::Status::NORMAL){
            this->ref_camera_frame_deque_.push_front(current_camera_frame);
            ref_camera_frame_latest_index = camera_frame_index;

            if(this->camera_frame_deque_.size()- (ref_camera_frame_latest_index + 1)<= this->max_num_fail_){
                this->status_ = Initializer::Status::EXTEND;  
            }

            if(this->ref_camera_frame_deque_.size()>this->max_num_ref_camera_frame_){
                this->status_ = Initializer::Status::SUCCESS;
                return;
            }
        }
        else{
            if(this->camera_frame_deque_.size()- (ref_camera_frame_latest_index + 1)<= this->max_num_fail_){

                this->status_ = Initializer::Status::EXTEND;  
            }
            else{
                this->status_ = Initializer::Status::FAIL; 
            }

        }

        camera_frame_index++;
    }

    

}

void Initializer::pipeline(std::shared_ptr<CameraFrame> &camera_frame){


    VLOG(VERBOSE) << "Initializer start with camera frame id: " << camera_frame->id_;


    this->detector_->pipeline(camera_frame);


    updateStatus(camera_frame);


    this->reconstructor_->pipeline(camera_frame);

    
    
    VLOG(VERBOSE) << "Initializer end with camera frame id: " << camera_frame->id_;
    


}

    
    
} //namespace modules_vins