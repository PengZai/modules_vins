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
#include "../data/frame.h"
#include "../data/map.h"
#include "../log/logging.h"
#include "../data/preprocess.h"
#include "../system/state.h"
#include "../visualization/visualizer.h"



namespace modules_vins{

class VisualFrontend{

    public:

    enum Status{
        NOT_INITIALIZED=-1,
        NORMAL=0,
        GET_LOST,
    };

    void printfStatus();

    inline const char* StatusToString(Status s) {
        switch (s) {
            case Status::NOT_INITIALIZED: return "NOT_INITIALIZED";
            case Status::NORMAL:             return "NORMAL";
            case Status::GET_LOST:             return "GET_LOST";
            default:                  return "UNKNOWN_STATUS";
        }
    }



    VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config);
    virtual ~VisualFrontend() = default;

    void updateMap(const Frame &frame);
    void pipeline(const std::shared_ptr<Frame> &frame);
    virtual void initPipeline(const std::shared_ptr<Frame> &frame) = 0;
    virtual void normalPipeline(const std::shared_ptr<Frame> &frame) = 0;
    void setRefFrameDeque(const std::deque<std::shared_ptr<Frame>> &ref_frame_deque);
    void maintainRefFrameDeque();
    Status getStatus();
    const std::deque<std::shared_ptr<Frame>> &getRefFrameDeque();
    void updateStautsInitialized2Normal();
    void checkInitializationAndUpdateStatusNotInitialized2Initialized(const std::shared_ptr<Frame> &frame);
    void propogateMappointWitchMatchInTimeRelationship(const std::shared_ptr<Frame> &frame);
    bool initializeComparisonPosewWithFrame(const std::shared_ptr<Frame> &frame);

    void setDataProprocesor(const std::shared_ptr<DataPreprocesor> &data_preprocesor);
    void setDetector(const std::shared_ptr<Detector> &detector);
    void setTracker(const std::shared_ptr<Tracker> &tracker);
    void setReconstructor(const std::shared_ptr<Reconstructor> &reconstructor);
    void setPoseEstimator(const std::shared_ptr<PoseEstimator> &pose_estimator);
    void setVisualizer(const std::shared_ptr<Visualizer> &visualizer);

    void setMap(const std::shared_ptr<Map> &map);
    void setState(const std::shared_ptr<State> &state);



    protected:
    int fail_pose_estimation_num_;

    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<State> state_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<DataPreprocesor> data_preprocesor_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::shared_ptr<Visualizer> visualizer_;
    std::deque<std::shared_ptr<Frame>> frame_deque_;
    std::deque<std::shared_ptr<Frame>> ref_frame_deque_;
    Status status_;



};
    


} // namespace modules_vins