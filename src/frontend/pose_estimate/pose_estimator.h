#pragma once

#include <opencv2/opencv.hpp>

#include <Eigen/Core>               
#include <opencv2/core/eigen.hpp>

#include "../../data/camera.h"
#include "../../system/system_config.h"

namespace modules_vins
{




class PoseEstimator{

    public:

    PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config);
    bool checkEstimatedPose(const Sophus::SE3<double> &Transformation,const int num_inliers);

    std::shared_ptr<SystemConfig> sys_config_;
    std::deque<CameraFrame> camera_frame_deque_;

   
    void pipeline(CameraFrame &camera_frame);


};


    
} // namespace modules_vins


