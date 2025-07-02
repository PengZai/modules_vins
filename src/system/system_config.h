#pragma once

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <typeinfo>

#include "config.h"
#include "camera_config.h"
#include "imu_config.h"
#include "visualizer_config.h"
#include "feature_and_tracker_config.h"
#include "comparison_config.h"
#include "lidar_config.h"

namespace modules_vins
{   


class SystemParameters: public Parameters{


    public:

        std::string log_verbosity_; // VERBOSE, KEY, QUIET
        int num_used_camera; // number of cameras, 2 for stereo, 1 for monocular
        int num_used_imu; // number of IMUs
        int num_used_lidar; // number of cameras, 2 for stereo, 1 for monocular
        int num_used_imu; // number of IMUs

        int maximum_num_fail_;
        int minimum_num_in_ref_camera_frame_;
        double minimum_cumulative_translation_;

        double threshold_for_tracking_descriptor_in_time_;
        double threshold_for_tracking_descriptor_in_frame_;
        double matching_ratio_;
        double max_stereo_time_offset_;
        double max_color_sensor_depth_pair_time_offset_;

        double threshold_for_pnp_pose_log_norm_;
        int min_inliers_;
        int max_num_backward_reference_;
        int max_fail_num_;
        int max_num_local_map_size_;

        double maximum_estimated_depth_;
        double minimum_estimated_depth_;


        double minimum_key_camera_frame_translation_;

        std::string feature_and_tracker_config_name_;

        std::string camera_config_name_;
        std::string lidar_config_name_;
        std::string imu_config_name_;
        std::string visualizer_config_name_;
        std::string comparison_config_name_;

        std::string model_path_;
        
        // robot_base, it have to be the name of each sensor, like imu0, cam0, cam1, or even comparison, like GT, VINSFUSION
        std::string robot_base_;


        bool use_comparison_pose_for_pose_estimation_;
        int comparison_pose_idx_for_pose_estimation_;

        std::string output_dir_;


        bool check_triangulation_;

    public:
        void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;



};

class SystemConfig: public Config
{

    
    public:

        SystemConfig();
        
        void loadFromPath(const std::string &config_path);

        template<typename SensorConfigType>
        void setSensorConfig(const std::shared_ptr<SensorConfigType> &sensor_config);
        void setVisualizerConfig(const std::shared_ptr<VisualizerConfig> &visualizer_config);
        void setFeatureAndTrackerConfig(const std::shared_ptr<FeatureAndTrackerConfig> feature_and_tracker_config);
        void setComparisonConfig(const std::shared_ptr<ComparisonConfig> comparison_config);
        void calculateTransformationsBetweenRobotBaseAndSensors();
        template<typename ConfigType, typename ParameterType>
        void loadSensorConfigAndParameters(const std::shared_ptr<ConfigType> &sensor_config, const std::string &config_path, const int num_used_sensors);

    
    public:

        std::shared_ptr<SystemParameters> params_ = nullptr;
        std::shared_ptr<VisualizerConfig> visualizer_config_ = nullptr;
        std::shared_ptr<FeatureAndTrackerConfig> feature_and_tracker_config_ = nullptr;

        // sensor configuration
        std::shared_ptr<CameraConfig> camera_config_ = nullptr;
        std::shared_ptr<ImuConfig> imu_config_ = nullptr;
        std::shared_ptr<LidarConfig> lidar_config_ = nullptr;
        std::shared_ptr<ComparisonConfig> comparison_config_ = nullptr;

        //sensor parameters
        std::map<std::string, std::shared_ptr<Parameters>> frame_name_to_sensor_parameters_ptr_map_;


    

};




} // namespace modules_vins


