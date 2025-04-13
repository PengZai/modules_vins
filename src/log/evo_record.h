#pragma once
#include<memory>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>

#include "logging.h"
#include "../system/system_config.h"


namespace modules_vins{



class EVORecorder{

    public:
    EVORecorder(const std::shared_ptr<SystemConfig> sys_config);

    void writeTrajectoryOnce(const double timestamp, const Eigen::Vector3d &t, const Eigen::Quaterniond &q);

    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::ofstream outfile_;


};



} // namespace modules_vins