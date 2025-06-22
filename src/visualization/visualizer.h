#pragma once

#include "../system/state.h"
#include "opencv_visualizer.h"
#include "pangolin_visualizer.h"
#include "ros1_visualizer.h"
#include "../log/logging.h"
#include "../system/system_config.h"
#include "../data/camera.h"


namespace modules_vins{

class Visualizer{
    
    
    public:

    Visualizer(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh);
    ~Visualizer();
    void setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh);
    void publish(const std::shared_ptr<CameraFrame> &camera_frame,  const State &state);
    void setMap(const std::shared_ptr<Map> &map);
    std::shared_ptr<OpenCVVisualizer> &getOpenCVVisualizer();


    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<OpenCVVisualizer> opencv_visualizer_;
    std::shared_ptr<PangolinVisualizer> pangolin_visualizer_;
    std::shared_ptr<ROS1Visualizer> ros1_visualizer_;

};


} //modules_vins
