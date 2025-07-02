#pragma once


#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "../utils/utils.h"


namespace modules_vins
{

class LidarParameters : public Parameters
{

    public:

        LidarParameters();

        void loadFromNode(const std::shared_ptr<cv::FileNode> &node);



    public:
        Eigen::Matrix4d T_imu0_lidar;
        Eigen::Matrix4d T_base_lidar;

        

};

class LidarConfig : public Config
{

    public:
        std::vector<std::shared_ptr<LidarParameters>> params_vector_;


}; 





} // namespace modules_vins