#include "tracker.h"


namespace modules_vins{

  
Tracker::Tracker(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;

}


    
void Tracker::pipeline(std::shared_ptr<CameraFrame> &camera_frame){


    std::shared_ptr<CameraFrame> &ref_camera_frame = camera_frame->ref_camera_frame_;

    if(ref_camera_frame == nullptr){

        LOG(INFO) << RED << "ref_camera_frame is nullptr" << RESET;
        return ;
    }

    std::shared_ptr<Image> &img0_from_current_frame = camera_frame->image_vector_.at(0);

    std::shared_ptr<Image> &img0_from_ref_frame = ref_camera_frame->image_vector_.at(0);
    

    // we track feature according to the feature in camera 0(left camera)
    trackInTime(img0_from_ref_frame, img0_from_current_frame);




}

    

    
    
} //modules_vins