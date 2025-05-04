#pragma once


#include <memory>

#include "../reconstruct/reconstructor.h"
#include "../detect/detector.h"
#include "../tracking/tracker.h"
#include "../pose_estimate/pose_estimator.h"

#include "../system/system_config.h"
#include "../system/state.h"
#include "../data/camera.h"
#include "../utils/utils.h"

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

    inline const char* StatusToString(Status s) {
        switch (s) {
            case Status::NOT_INITIALIZED_YET: return "NOT_INITIALIZED_YET";
            case Status::WORKING:             return "WORKING";
            case Status::EXTEND:              return "EXTEND";
            case Status::FAIL:                return "FAIL";
            case Status::SUCCESS:             return "SUCCESS";
            default:                  return "UNKNOWN_STATUS";
        }
    }


    Initializer(const std::shared_ptr<SystemConfig> sys_config);

    bool initializeGTTcwWithCameraFrame(const std::shared_ptr<CameraFrame> &camera_frame, State &state);
    void pipeline(std::shared_ptr<CameraFrame> &camera_frame);
    Status getStatus();
    void updateStatus(std::shared_ptr<CameraFrame> &camera_frame);
    bool checkSuccess();
    void printfStatus();
    const std::deque<std::shared_ptr<CameraFrame>> &getInitializedReferenceCameraFrameDeque() const;
    double getCumulativeTranslationInCameraFrameDeque(const std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque);
    
    std::shared_ptr<SystemConfig> sys_config_;

    protected:
    Sophus::SE3<double> T_cam_GT_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> trakcer_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::deque<std::shared_ptr<CameraFrame>> camera_frame_deque_;
    int origin_index_in_camera_frame_deque_;
    int camera_frame_index_;
    int index_in_camera_frame_deque_for_latest_ref_camera_frame_; // a index in camera frame deque but its value was store in the latest element of ref_camera_frame_deque_
    std::deque<std::shared_ptr<CameraFrame>> ref_camera_frame_deque_;
    int num_fail_;
    int maximum_num_fail_;
    int minimum_num_ref_camera_frame_;
    double minimum_cumulative_translation_for_initialization_;

    Status status_;
};


} //namespace modules_vins