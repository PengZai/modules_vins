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



    Initializer(const std::shared_ptr<SystemConfig> sys_config);
    ~Initializer() = default;
    bool initializeGTTcwWithCameraFrame(const std::shared_ptr<CameraFrame> &camera_frame, const std::shared_ptr<State> &state);

    std::shared_ptr<SystemConfig> sys_config_;




};


} //namespace modules_vins