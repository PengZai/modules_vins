#pragma once


#include<memory>

#include "../detect/detector.h"
#include "../data/camera.h"
#include "../system/system_config.h"
#include "../reconstruct/two_view_depths/two_view_reconstruction.h"


namespace modules_vins{

class Tracker{

    public:

    Tracker(const std::shared_ptr<SystemConfig> &sys_config);
    virtual ~Tracker() = default;

    void trackInFrame(const std::shared_ptr<CameraFrame> &camera_frame);
    void trackInTime(const std::shared_ptr<Image> &img_from_ref_frame, const std::shared_ptr<Image> &img_from_current_frame);
    virtual void matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, 
        std::vector<cv::DMatch> &good_matches, 
        const float error_threshold = -1.0, const float y_distance_threshold = -1.0) = 0;
    void pipeline(const std::shared_ptr<CameraFrame> &camera_frame);
    void setDetector(const std::shared_ptr<Detector> &detector);


    protected:

    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<Detector> detector_;

};


} //modules_vins