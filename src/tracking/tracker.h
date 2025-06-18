#pragma once


#include<memory>

#include "../data/camera.h"
#include "../system/system_config.h"
#include "../reconstruct/two_view_depths/two_view_reconstruction.h"


namespace modules_vins{

class Tracker{

    public:

    Tracker(const std::shared_ptr<SystemConfig> &sys_config);
    virtual ~Tracker() = default;

    virtual void trackInFrame(std::shared_ptr<CameraFrame> &camera_frame) = 0;
    virtual void trackInTime(std::shared_ptr<Image> &img_from_ref_frame, std::shared_ptr<Image> &img_from_current_frame) = 0;
    void pipeline(std::shared_ptr<CameraFrame> &camera_frame);



    protected:

    std::shared_ptr<SystemConfig> sys_config_;

};


} //modules_vins