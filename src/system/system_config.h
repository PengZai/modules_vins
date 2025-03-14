#pragma once

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "camera_config.h"
#include "imu_config.h"

namespace modules_vins
{   


class SystemParameters: public Parameters{


    public:

        std::string log_verbosity_; // VERBOSE, KEY, QUIET
        int max_cameras_; // number of cameras, 2 for stereo, 1 for monocular
        int max_imus_; // number of IMUs

        int num_feature_points_;
        double max_stereo_time_offset_;


        std::string imu_config_name_;
        std::string camera_config_name_;

        // std::shared_ptr<ImuParameters> vector_imu_params_;


    public:
        void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;

};

class SystemConfig: public Config
{

    
    public:

        SystemConfig();
        
        void loadFromPath(const std::string &config_path);
        void loadConfigFromPath(const std::string &config_path) override;

        void setCameraConfig(const std::shared_ptr<CameraConfig> camera_config);

    
    public:

        std::shared_ptr<SystemParameters> params_ = nullptr;
        std::shared_ptr<CameraConfig> camera_config_ = nullptr;

    

};




} // namespace modules_vins


