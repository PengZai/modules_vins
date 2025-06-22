#include "KLT_frontend.h"



namespace modules_vins{


KLTFrontend::KLTFrontend(const std::shared_ptr<SystemConfig> &sys_config):
VisualFrontend(sys_config)
{


    

}


void KLTFrontend::initPipeline(const std::shared_ptr<CameraFrame> &camera_frame){


    LOG(INFO) << "Initializer start with camera frame id: " << camera_frame->id_;

    this->data_preprocesor_->pipeline(camera_frame);



    for(size_t i = 0; i < (int)this->ref_camera_frame_deque_.size(); i++){


        const std::shared_ptr<CameraFrame> &ref_camera_frame = this->ref_camera_frame_deque_[i];

        ref_camera_frame->cleanTrackInTimeRelationship();

        camera_frame->ref_camera_frame_ = ref_camera_frame;

        this->tracker_->pipeline(camera_frame);
        this->pose_estimator_->pipeline(camera_frame);

        // std::shared_ptr<OpenCVVisualizer> & opencv_visualizer = this->visualizer_->getOpenCVVisualizer();
        // opencv_visualizer->publishMatchingInTime(camera_frame);
        // cv::waitKey(0);
        

         if(camera_frame->status_ == CameraFrame::NORMAL){
            this->status_ = Status::NORMAL;
            this->ref_camera_frame_deque_.clear();
            this->detector_->pipeline(camera_frame);
            this->reconstructor_->pipeline(camera_frame);
            this->ref_camera_frame_deque_.push_back(camera_frame);
            this->ref_camera_frame_deque_.push_back(camera_frame->ref_camera_frame_);
            return;
        }

    }

    this->detector_->pipeline(camera_frame);
    this->reconstructor_->pipeline(camera_frame);



    this->ref_camera_frame_deque_.push_front(camera_frame);
    if(this->ref_camera_frame_deque_.size()>20){
        this->ref_camera_frame_deque_.pop_back();
    }
    
    this->printfStatus();



    LOG(INFO) << "Initializer end with camera frame id: " << camera_frame->id_;

}



void KLTFrontend::normalPipeline(const std::shared_ptr<CameraFrame> &camera_frame){


    LOG(INFO) << "VisualFrontend Start with camera frame id: " << camera_frame->id_;

    this->data_preprocesor_->pipeline(camera_frame);

    std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);


    if(camera_frame->id_ == 31){
            LOG(INFO) << "just test";
    }
    
    for(size_t i=0; i<this->ref_camera_frame_deque_.size();i++){

        
        std::shared_ptr<CameraFrame> &ref_camera_frame = ref_camera_frame_deque_.at(i);
        LOG(INFO) << "estimating pose betweeen reference frame with id: " << ref_camera_frame->id_ << " and current frame with id: " << camera_frame->id_;
        camera_frame->ref_camera_frame_ = ref_camera_frame;
        int ref_frame_matches_in_time_size = camera_frame->ref_camera_frame_->image_vector_.at(0)->matches_in_time_.size();
        if(ref_camera_frame->id_ == 26){
            LOG(INFO) << "just test";
        }

        this->tracker_->pipeline(camera_frame);
        this->pose_estimator_->pipeline(camera_frame);

        if(camera_frame->status_ == CameraFrame::Status::NORMAL){
            break;
        }
        else{
            LOG(INFO) << YELLOW << " Fail pose estimation with reference camera frame id : " << ref_camera_frame->id_ << " for camera frame id : " << camera_frame->id_ << RESET;
            
            // std::shared_ptr<OpenCVVisualizer> & opencv_visualizer = this->visualizer_->getOpenCVVisualizer();
            // opencv_visualizer->publishMatchingInTime(camera_frame);
            // cv::waitKey(0);
            // cv::destroyAllWindows();
            // LOG(INFO) << "just test";
        }

    }

    if(camera_frame->status_ == CameraFrame::Status::NORMAL){
        propogateMappointWitchMatchRelationship(camera_frame);
        this->ref_camera_frame_deque_.push_front(camera_frame);
        maintainRefCameraFrameDeque();
        this->fail_pose_estimation_num_=0;
    }
    else{
        LOG(INFO) << YELLOW << " Fail pose estimation for camera frame id : " << camera_frame->id_  << ", discard it"<< RESET;
        this->fail_pose_estimation_num_++;
        if(fail_pose_estimation_num_ > this->sys_config_->params_->maximum_num_fail_){

            this->status_ = Status::GET_LOST;
        }
    }

    this->detector_->pipeline(camera_frame);
    this->reconstructor_->pipeline(camera_frame);

    LOG(INFO) << "VisualFrontend End with camera frame id: " << camera_frame->id_;

}





} //modules_vins
