#pragma once
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <memory>

#include "../system/system_config.h"
#include "../data/camera.h"

namespace modules_vins
{


class PangolinVisualizer{


    public:

    PangolinVisualizer(const std::shared_ptr<SystemConfig> &sys_config);
    void publish(const CameraFrame &camera_frame);
    void drawFrame(const Eigen::Matrix4d &Twc);
    void drawMapPoints(const CameraFrame &camera_frame);

    public:

    std::shared_ptr<SystemConfig> sys_config_;

    pangolin::Var<bool> *is_reset_ = nullptr;
    pangolin::Var<bool> *is_follow_camera_ = nullptr;

    pangolin::View d_cam_;
    pangolin::OpenGlRenderState s_cam_;


    double frame_size_;
    double point_size_;
    double viewer_eye_positionX_;
    double viewer_eye_positionY_;
    double viewer_eye_positionZ_;
};

    
} // namespace modules_vins

