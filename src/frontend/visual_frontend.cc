#include "visual_frontend.h"

namespace modules_vins{


VisualFrontend::VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config),
fail_pose_estimation_num_(0),
status_(Status::NOT_INITIALIZED)
{


}


VisualFrontend::Status VisualFrontend::getStatus(){

    return this->status_;
}


const std::deque<std::shared_ptr<Frame>> &VisualFrontend::getRefFrameDeque(){
    return this->ref_frame_deque_;
}

void VisualFrontend::printfStatus(){

    LOG(INFO) << GREEN << "Initializer::Status: " << StatusToString(this->status_) << RESET;
    LOG(INFO) << GREEN << "ref_frame_deque_.size : " << this->ref_frame_deque_.size() << RESET;

}



void VisualFrontend::setDataProprocesor(const std::shared_ptr<DataPreprocesor> &data_preprocesor){
    this->data_preprocesor_ = data_preprocesor;
}

void VisualFrontend::setDetector(const std::shared_ptr<Detector> &detector){

    this->detector_ = detector;
}

void VisualFrontend::setTracker(const std::shared_ptr<Tracker> &tracker){

    this->tracker_ = tracker;
}

void VisualFrontend::setReconstructor(const std::shared_ptr<Reconstructor> &reconstructor){

    this->reconstructor_ = reconstructor;
}

void VisualFrontend::setPoseEstimator(const std::shared_ptr<PoseEstimator> &pose_estimator){

    this->pose_estimator_ = pose_estimator;
}

void VisualFrontend::setVisualizer(const std::shared_ptr<Visualizer> &visualizer){
    this->visualizer_ = visualizer;
}

void VisualFrontend::setRefFrameDeque(const std::deque<std::shared_ptr<Frame>> &ref_frame_deque){

    this->ref_frame_deque_ = ref_frame_deque;
    
}


void VisualFrontend::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;
}


void VisualFrontend::setState(const std::shared_ptr<State> &state){
    this->state_ = state;
}

void VisualFrontend::maintainRefFrameDeque(){

    double culmulative_trans = 0;
    int start_erase_index = -1;
    if(this->ref_frame_deque_.size() > 0){

        double Tbw_translation_norm = this->ref_frame_deque_.front()->T_b_w_.translation().norm();
        for(size_t i=1;i<this->ref_frame_deque_.size();i++){

            const std::shared_ptr<Frame> &prev_frame = this->ref_frame_deque_.at(i);

            double prev_Tbw_translation_norm = prev_frame->T_b_w_.translation().norm();
            culmulative_trans  += Tbw_translation_norm - prev_Tbw_translation_norm;
            Tbw_translation_norm = prev_Tbw_translation_norm;

            if(culmulative_trans > this->sys_config_->params_->minimum_cumulative_translation_){
                start_erase_index = i;
                break;
            }
        }

        if(start_erase_index != -1){
            ref_frame_deque_.erase(ref_frame_deque_.begin() + start_erase_index, ref_frame_deque_.end());
        }

    }
    


}

bool VisualFrontend::initializeGTTcwWithFrame(const std::shared_ptr<Frame> &frame){

    const std::shared_ptr<Image> &img_0 = frame->image_vector_.at(0);

    if(!img_0){
        return false;
    }

    this->state_->SynchronizeAndTransformComparisonPoseToRobotBaseCoordinate(img_0->timestamp_);

    return true;
}


void VisualFrontend::pipeline(const std::shared_ptr<Frame> &frame){


        switch (this->status_) {

            case Status::NOT_INITIALIZED:
                initPipeline(frame);
                break;
            case Status::NORMAL:
                normalPipeline(frame);
                break;
            case Status::GET_LOST:
                break;
        }
    
}




} //modules_vins

