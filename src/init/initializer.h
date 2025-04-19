#pragma once


#include <memory>

#include "../reconstruct/reconstructor.h"
#include "../detect/detector.h"
#include "../tracking/tracker.h"
#include "../pose_estimate/pose_estimator.h"

#include "../system/system_config.h"
#include "../system/state.h"
#include "../data/camera.h"

namespace modules_vins{

class Initializer{

    public:

    enum Status{
        NOT_INITIALIZED_YET=-1, // when we get the first camera frame
        WORKING=0, //looking for suitable camera frame with ref_camera_frame_deque_ in forwoard.
        EXTEND, // there are not enough number of frame but still less than maximum fail number.
        FAIL, //fail in collecting enough number of reference frame in according to the origin, reset the origin to the next frame of previous origin.
        SUCCESS, // success for initialization process
    };


    Initializer(const std::shared_ptr<SystemConfig> sys_config);

    bool initializeGTTcwWithCameraFrame(std::shared_ptr<CameraFrame> &camera_frame, State &state);
    void pipeline(std::shared_ptr<CameraFrame> &camera_frame);
    Status getStatus();
    void updateStatus(std::shared_ptr<CameraFrame> &camera_frame);

    std::shared_ptr<SystemConfig> sys_config_;

    protected:
    Sophus::SE3<double> T_cam_GT_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> trakcer_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::deque<std::shared_ptr<CameraFrame>> camera_frame_deque_;
    int origin_index_;
    std::deque<std::shared_ptr<CameraFrame>> ref_camera_frame_deque_;
    int ref_camera_frame_index_;
    int num_fail_;
    int max_num_fail_;
    int max_num_ref_camera_frame_;

    Status status_;
};


} //namespace modules_vins