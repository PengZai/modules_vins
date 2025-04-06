#pragma once

#include <memory>
#include <Eigen/Dense>

// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/viz.hpp> 

#include <pangolin/pangolin.h>


#include "../system/state.h"
#include "../system/system_config.h"
#include "../data/camera.h"

namespace modules_vins
{


class PangolinVisualizer{


    public:

    PangolinVisualizer(const std::shared_ptr<SystemConfig> &sys_config);
    void publish(const State& state);
    void drawFrame(const Eigen::Matrix4d &Twc);
    void drawMapPoints(const State& state);
    void drawTrajectory(const State &state);

    public:

    std::shared_ptr<SystemConfig> sys_config_;


    // cv::viz::Viz3d *vis;

    pangolin::Var<bool> *is_reset_ = nullptr;
    pangolin::Var<bool> *is_follow_camera_ = nullptr;

    pangolin::View d_cam_;
    pangolin::OpenGlRenderState s_cam_;


    double frame_size_;
    double point_size_;
    double trajectory_line_size_;
    double viewer_eye_positionX_;
    double viewer_eye_positionY_;
    double viewer_eye_positionZ_;

    Eigen::Vector3d ref_translation_;

};

    
} // namespace modules_vins

