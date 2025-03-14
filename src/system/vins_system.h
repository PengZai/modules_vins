#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>

#include "../data/camera.h"
#include "system_config.h"
#include "../frontend/visual_frontend.h"
#include "../visualization/visualizer.h"


namespace modules_vins
{
class System {
    public:
        System();

        void setConfig(const std::shared_ptr<SystemConfig> &config);
        void setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh);
        void setVisualFrontend(const std::shared_ptr<VisualFrontend> &visual_frontend);
        void setVisualizer(const std::shared_ptr<Visualizer> &visualizer);

        // msg0 and msg1 come from camera 0 and camera 1 respectively, 
        // in which msg0 and msg1 have been software synchronized
        void addCameraFrameDeque(const std::vector<rosbag::MessageInstance> &msgs);

        void callbackVisualNavigation();


    public:
        std::shared_ptr<SystemConfig> config_;
  



    protected:

        std::shared_ptr<VisualFrontend> visual_frontend_;
        std::shared_ptr<Visualizer> visualizer_;
        std::deque<CameraFrame> camera_frame_deque_;
        std::atomic<bool> is_thread_running_;
        std::shared_ptr<ros::NodeHandle> nh_;



};


} // namespace modules_vins
