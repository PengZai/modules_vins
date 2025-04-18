#pragma once


#include<memory>

#include "../../data/camera.h"
#include "../../system/system_config.h"
#include "tracking_descriptor.h"
#include "../reconstruct/two_view_depths/two_view_reconstruction.h"


namespace modules_vins{

class Tracker{

    public:

    Tracker(const std::shared_ptr<SystemConfig> &sys_config);
    void trackInFrame(std::shared_ptr<CameraFrame> &camera_frame);
    void trackInTime(std::shared_ptr<Image> &img_from_ref_frame, std::shared_ptr<Image> &img_from_current_frame);
    void pipeline(std::shared_ptr<CameraFrame> &ref_camera_frame, std::shared_ptr<CameraFrame> &camera_frame);



    protected:

    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<BFMatcher> bf_;

};


} //modules_vins