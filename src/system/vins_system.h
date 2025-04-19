#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>

#include "../frontend/visual_frontend.h"
#include "../visualization/visualizer.h"
#include "../data/camera.h"
#include "system_config.h"
#include "../init/initializer.h"
#include "../log/logging.h"
#include "../log/evo_record.h"
#include "state.h"


namespace modules_vins
{





class System {
    public:
        System();

        void setConfig(const std::shared_ptr<SystemConfig> &config);
        void setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh);
        void setInitializer(const std::shared_ptr<Initializer> &initializer);
        void setVisualFrontend(const std::shared_ptr<VisualFrontend> &visual_frontend);
        void setVisualizer(const std::shared_ptr<Visualizer> &visualizer);
        void setMap(const std::shared_ptr<Map> &map);
        void setRecorder(const std::shared_ptr<EVORecorder> &evo_recorder);
        void pushbackGTState(const Sophus::SE3<double> &GT_T_c_w);
        void GT2CameraCoordinateAndSet(const std::map<double, Sophus::SE3<double>> &timestamp_GT_T_full_map);
        void setGTState(const std::map<double, Sophus::SE3<double>> timestamp_GT_T_c_w_map);

        void RosMessagePtrToCvImageConstPtr(std::shared_ptr<rosbag::MessageInstance> &msg_ptr, cv_bridge::CvImageConstPtr &cv_ptr, const std::string &to_cv_dtype);
        void RosMessagePtrToCvImageConstPtr(std::shared_ptr<rosbag::MessageInstance> &msg_ptr, cv_bridge::CvImageConstPtr &cv_ptr);
        
        const State &getState() const;
        void updateState(const std::shared_ptr<CameraFrame> &camera_frame);
        // msg0 and msg1 come from camera 0 and camera 1 respectively, 
        // in which msg0 and msg1 have been software synchronized
        void addCameraFrameDeque(const std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>> &msg_groups);

        void callbackVisualNavigation();


    public:
        std::shared_ptr<SystemConfig> config_;
  



    protected:
        std::shared_ptr<Initializer> initializer_;
        std::shared_ptr<VisualFrontend> visual_frontend_;
        std::shared_ptr<Visualizer> visualizer_;
        std::deque<std::shared_ptr<CameraFrame>> camera_frame_deque_;
        std::atomic<bool> is_thread_running_;
        std::shared_ptr<ros::NodeHandle> nh_;
        std::shared_ptr<EVORecorder> evo_recorder_;

        State state_;


        enum Status{
            NOT_INITIALIZED=-1,
            NORMAL=0,
            LOST,
        };
    
        Status status_;






};


} // namespace modules_vins
