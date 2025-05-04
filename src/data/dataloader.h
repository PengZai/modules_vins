#pragma once

#include <iostream>
#include <string>
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// #include <glog/logging.h>
#include <sensor_msgs/Image.h>
#include <limits>
#include <fstream>


#include "../log/logging.h"
#include "../system/system_config.h"
#include "map.h"
#include "../system/state.h"


namespace modules_vins
{


class FileDataLoader{

    public:
    FileDataLoader(const std::shared_ptr<SystemConfig> &sys_config);
    void load_groundtruth(const std::string &path_to_file, std::map<double, Sophus::SE3<double>> &timestamp_GT_T_map);

    public:
    std::shared_ptr<SystemConfig> sys_config_;
};


class ROSDataLoader{

    public:
    ROSDataLoader(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh);
    int findSynchronizedIndex(const std::string &rostopic, int m_source_index, double max_tolerant_time_offset);


    public:
    std::shared_ptr<SystemConfig> sys_config_;

    std::shared_ptr<ros::NodeHandle> nh_;


    std::string path_to_bag_;

    rosbag::Bag bag_;
    rosbag::View view_full_;

    double max_camera_time_;

    std::vector<rosbag::MessageInstance> loaded_msgs_;
    std::vector<std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>>> msg_groups_ready_for_process_;

    void load_data(const std::string &path_to_bag, std::vector<std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>>> &msg_groups_ready_for_process);

};
    
} // namespace modules_vins



