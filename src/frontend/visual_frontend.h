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
    virtual ~VisualFrontend() = default;

    void updateMap(const CameraFrame &camera_frame);
    virtual void pipeline(std::shared_ptr<CameraFrame> &camera_frame) = 0;
    void setRefCameraFrameDeque(const std::deque<std::shared_ptr<CameraFrame>> &ref_camera_frame_deque);
    void maintainRefCameraFrameDeque();
    Status getStatus();

    void setDetector(const std::shared_ptr<Detector> &detector);
    void setTracker(const std::shared_ptr<Tracker> &tracker);
    void setReconstructor(const std::shared_ptr<Reconstructor> &reconstructor);
    void setPoseEstimator(const std::shared_ptr<PoseEstimator> &pose_estimator);
    void setMap(const std::shared_ptr<Map> &map);




    protected:
    int fail_pose_estimation_num_;
    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::deque<std::shared_ptr<CameraFrame>> camera_frame_deque_;
    std::deque<std::shared_ptr<CameraFrame>> ref_camera_frame_deque_;
    Status status_;



};
    


} // namespace modules_vins