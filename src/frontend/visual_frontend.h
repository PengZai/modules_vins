#pragma once

#include<vector>
#include<memory>
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>

#include "reconstruct/reconstructor.h"
#include "detect/detector.h"
#include "../system/system_config.h"
#include "../data/camera.h"
#include "../data/map.h"
#include "tracking/tracker.h"
#include "pose_estimate/pose_estimator.h"
#include "../log/logging.h"





namespace modules_vins{

class VisualFrontend{

    public:

    VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config);
    void setMap(const std::shared_ptr<Map> &map);
    void updateMap(const CameraFrame &camera_frame);
    void pipeline(std::shared_ptr<CameraFrame> &camera_frame);

    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> trakcer_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::deque<std::shared_ptr<CameraFrame>> camera_frame_deque_;
    std::shared_ptr<CameraFrame> ref_camera_frame_;


    enum VisualFrontendStatus{
        NOT_INITIALIZED=-1,
        NORMAL=0,
        LOST_TRACKING,
        FAIL_POSE_ESTIMATION,
    };

    VisualFrontendStatus status_;


};
    


} // namespace modules_vins