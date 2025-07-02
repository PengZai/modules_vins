#include "KLT_frontend.h"



namespace modules_vins{


KLTFrontend::KLTFrontend(const std::shared_ptr<SystemConfig> &sys_config):
VisualFrontend(sys_config)
{


    

}


void KLTFrontend::initPipeline(const std::shared_ptr<Frame> &frame){


    LOG(INFO) << "Initializer start with camera frame id: " << frame->id_;

    this->data_preprocesor_->pipeline(frame);

    std::shared_ptr<Image> &img_0 = frame->image_vector_.at(0);

    int num_inliner = -1;

    for(size_t i = 0; i < this->ref_frame_deque_.size(); i++){


        const std::shared_ptr<Frame> &ref_frame = this->ref_frame_deque_[i];

        ref_frame->cleanTrackInTimeRelationship();

        frame->ref_frame_ = ref_frame;

        this->tracker_->pipeline(frame);
        double maximum_motion_norm = (frame->id_ - ref_frame->id_) * this->sys_config_->params_->threshold_for_pnp_pose_log_norm_;

        bool success = this->pose_estimator_->pipeline(frame, num_inliner, maximum_motion_norm);

        // std::shared_ptr<OpenCVVisualizer> & opencv_visualizer = this->visualizer_->getOpenCVVisualizer();
        // opencv_visualizer->publishMatchingInTime(frame);
        // cv::waitKey(0);
        

         if(frame->status_ == Frame::NORMAL){
            this->status_ = Status::NORMAL;
            initializeGTTcwWithFrame(frame);
            frame->propogateMappointWitchMatchInTimeRelationship();
            if(frame->use_comparison_pose_for_pose_estimation_){
                bool success = state_->getPoseComparisonForTbwWithIdx(frame->timestamp_, 
                this->sys_config_->params_->comparison_pose_idx_for_pose_estimation_, 
                frame->T_b_w_);

                // frame->setTcwWithCamera0(frame->T_b_w_);
                // frame->setCamera0VelocityWithCamera0Tcw();
            }

            break;
        }

    }

    if (img_0->keypoint_vector_.size() < this->sys_config_->feature_and_tracker_config_->klt_params_->num_feature_points_) {
        this->detector_->pipeline(frame);
        this->reconstructor_->pipeline(frame);
    }




    this->ref_frame_deque_.push_front(frame);
    if(this->ref_frame_deque_.size()>20){
        this->ref_frame_deque_.pop_back();
    }
    
    this->printfStatus();



    LOG(INFO) << "Initializer end with camera frame id: " << frame->id_;

    if(this->status_ == Status::NORMAL){
        this->ref_frame_deque_.clear();
        this->ref_frame_deque_.push_back(frame);
        this->ref_frame_deque_.push_back(frame->ref_frame_);
    }

}



void KLTFrontend::normalPipeline(const std::shared_ptr<Frame> &frame){


    LOG(INFO) << "VisualFrontend Start with camera frame id: " << frame->id_;

    this->data_preprocesor_->pipeline(frame);

    std::shared_ptr<Image> &img_0 = frame->image_vector_.at(0);


    int num_inliner = 0;
    
    for(size_t i=0; i<this->ref_frame_deque_.size();i++){

        std::shared_ptr<Frame> &latest_ref_frame = ref_frame_deque_.at(0);
        std::shared_ptr<Frame> &ref_frame = ref_frame_deque_.at(i);
        double relative_motion_norm_between_latest_and_ref = (latest_ref_frame->T_b_w_ * ref_frame->T_b_w_.inverse()).log().norm();
        double maximum_motion_norm = relative_motion_norm_between_latest_and_ref + this->sys_config_->params_->threshold_for_pnp_pose_log_norm_;

        LOG(INFO) << "estimating pose betweeen reference frame with id: " << ref_frame->id_ << " and current frame with id: " << frame->id_;
        frame->ref_frame_ = ref_frame;
        int ref_frame_matches_in_time_size = frame->ref_frame_->image_vector_.at(0)->matches_in_time_.size();

        frame->initializeTbwWithVelocity();

        this->tracker_->pipeline(frame);
        if(frame->use_comparison_pose_for_pose_estimation_){
            bool success = state_->getPoseComparisonForTbwWithIdx(frame->timestamp_, 
            this->sys_config_->params_->comparison_pose_idx_for_pose_estimation_, 
            frame->T_b_w_);

            // frame->setTcwWithCamera0(frame->image_vector_.at(0)->T_c_w_);
            // frame->setCamera0VelocityWithCamera0Tcw();
        }
        else{

            bool success = this->pose_estimator_->pipeline(frame, num_inliner, maximum_motion_norm);
        }


        if(frame->status_ == Frame::Status::NORMAL){
            break;
        }
        else{
            LOG(INFO) << YELLOW << " Fail pose estimation with reference camera frame id : " << ref_frame->id_ << " for camera frame id : " << frame->id_ << RESET;
            
            // std::shared_ptr<OpenCVVisualizer> & opencv_visualizer = this->visualizer_->getOpenCVVisualizer();
            // opencv_visualizer->publishMatchingInTime(frame);
            // cv::waitKey(0);
            // cv::destroyAllWindows();
            // LOG(INFO) << "just test";
        }

    }

    if(frame->status_ == Frame::Status::NORMAL){
        frame->propogateMappointWitchMatchInTimeRelationship();
        this->ref_frame_deque_.push_front(frame);
        maintainRefFrameDeque();
        this->fail_pose_estimation_num_=0;
    }
    else{
        LOG(INFO) << YELLOW << " Fail pose estimation for camera frame id : " << frame->id_  << ", discard it"<< RESET;
        this->fail_pose_estimation_num_++;
        if(fail_pose_estimation_num_ > this->sys_config_->params_->maximum_num_fail_){

            this->status_ = Status::GET_LOST;
        }
        return;
    }

    if (img_0->keypoint_vector_.size() < this->sys_config_->feature_and_tracker_config_->klt_params_->num_feature_points_) {

        this->detector_->pipeline(frame);
        this->reconstructor_->pipeline(frame);
    }

    LOG(INFO) << "VisualFrontend End with camera frame id: " << frame->id_;

}





} //modules_vins
