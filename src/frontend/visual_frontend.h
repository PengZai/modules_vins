#pragma once

#include<vector>
#include<memory>
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>

#include "../reconstruct/reconstructor.h"
#include "../detect/detector.h"
#include "../tracking/tracker.h"
#include "../pose_estimate/pose_estimator.h"
#include "../system/system_config.h"
#include "../utils/utils.h"
#include "../data/camera.h"
#include "../data/map.h"
#include "../log/logging.h"





namespace modules_vins{

class VisualFrontend{

    public:

    enum Status{
        NOT_INITIALIZED=-1,
        NORMAL=0,
        LOST_TRACKING,
        FAIL_POSE_ESTIMATION,
        GET_LOST,
    };


    VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config);
    void setMap(const std::shared_ptr<Map> &map);
    void updateMap(const CameraFrame &camera_frame);
    void pipeline(std::shared_ptr<CameraFrame> &camera_frame);
    void setRefCameraFrameDeque(const std::deque<std::shared_ptr<CameraFrame>> &ref_camera_frame_deque);
    void maintainRefCameraFrameDeque();
    Status getStatus();


    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> trakcer_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::deque<std::shared_ptr<CameraFrame>> camera_frame_deque_;
    std::deque<std::shared_ptr<CameraFrame>> ref_camera_frame_deque_;
    int index_in_camera_frame_deque_for_latest_ref_camera_frame_;  
    int maximum_num_fail_;
    double minimum_cumulative_translation_for_visual_frontend_;
    Status status_;



};
    


} // namespace modules_vins