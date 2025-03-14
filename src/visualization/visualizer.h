#pragma once

#include "opencv_visualizer.h"
#include "ros1_visualizer.h"
#include "../log/logging.h"
#include "../system/system_config.h"
#include "../data/camera.h"


namespace modules_vins{

class Visualizer{
    
    
    public:

    Visualizer(const std::shared_ptr<SystemConfig> &config, const std::shared_ptr<ros::NodeHandle> &nh);
    ~Visualizer();
    void setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh);
    void publish(const CameraFrame &camera_frame);

    public:
    std::shared_ptr<SystemConfig> config_;
  
    std::shared_ptr<OpenCVVisualizer> opencv_visualizer_;
    std::shared_ptr<ROS1Visualizer> ros1_visualizer_;

};


} //modules_vins
