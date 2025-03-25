#pragma once

#include<vector>
#include<memory>
#include <Eigen/Dense>
#include<opencv2/opencv.hpp>

#include "../system/system_config.h"
#include "../data/camera.h"
#include "../data/map.h"
#include "detect/detector.h"
#include "tracking/tracker.h"
#include "reconstruct/reconstructor.h"
#include "pose_estimate/pose_estimator.h"



namespace modules_vins{

class VisualFrontend{

    public:

    VisualFrontend(const std::shared_ptr<SystemConfig> &sys_config);
    void setMap(const std::shared_ptr<Map> &map);
    void updateMap(const CameraFrame &camera_frame);
    void pipeline(CameraFrame &camera_frame);

    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Tracker> trakcer_;
    std::shared_ptr<Reconstructor> reconstructor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;


};
    


} // namespace modules_vins