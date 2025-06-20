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
#include "../data/preprocess.h"


namespace modules_vins{

class Initializer{

    public:

    enum Status{
        NOT_INITIALIZED_YET=-1, // when we get the first camera frame
        SUCCESS, // success for initialization process
    };

    inline const char* StatusToString(Status s) {
        switch (s) {
            case Status::NOT_INITIALIZED_YET: return "NOT_INITIALIZED_YET";
            case Status::SUCCESS:             return "SUCCESS";
            default:                  return "UNKNOWN_STATUS";
        }
    }


    Initializer(const std::shared_ptr<SystemConfig> sys_config);
    virtual ~Initializer() = default;

    bool initializeGTTcwWithCameraFrame(const std::shared_ptr<CameraFrame> &camera_frame, State &state);
    Status getStatus();
    void updateStatus(const std::shared_ptr<CameraFrame> &latest_camera_frame);
    void printfStatus();
    const std::deque<std::shared_ptr<CameraFrame>> &getGoodInitializedReferenceCameraFrameDeque() const;
    double getCumulativeTranslationInCameraFrameDeque(const std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque);
    
    void setDataProprocesor(const std::shared_ptr<DataPreprocesor> &data_preprocesor);
    void setDetector(const std::shared_ptr<Detector> &detector);
    void setTracker(const std::shared_ptr<Tracker> &tracker);
    void setReconstructor(const std::shared_ptr<Reconstructor> &reconstructor);
    void setPoseEstimator(const std::shared_ptr<PoseEstimator> &pose_estimator);

    //interface 
    virtual void pipeline(const std::deque<std::shared_ptr<CameraFrame>> &camera_frame_deque) = 0;


    std::shared_ptr<SystemConfig> sys_config_;



    protected:
    Sophus::SE3<double> T_cam_GT_;
    std::shared_ptr<DataPreprocesor> data_preprocesor_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;

    std::deque<std::shared_ptr<CameraFrame>> ref_camera_frame_deque_;
    std::deque<std::shared_ptr<CameraFrame>> good_ref_camera_frame_deque_;


    Status status_;
};


} //namespace modules_vins