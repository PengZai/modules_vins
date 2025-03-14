#pragma once

#include <iostream>
#include <glog/logging.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "colors.h"




#define QUIET   0
#define KEY     1
#define VERBOSE 2



class Logger{
    public:
        static void setLogger(const char* const *argv, const std::string &LogVerbosity);
        static void setLogVerbosity(const std::string &Verbosity);

        static google::LogSeverity SERVERBILITY_;
        static std::unordered_map<std::string, int> LOG_LEVEL_MAP_;

};


// namespace modules_vins{
// class System {
//     public:
//         System(const std::shared_ptr<cv::FileStorage> &config, const std::shared_ptr<ros::NodeHandle> &nh);


//     std::shared_ptr<ros::NodeHandle> nh;
//     std::shared_ptr<cv::FileStorage> config;



// };
// } // namespace modules_vins

