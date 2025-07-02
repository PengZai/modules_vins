#pragma once


#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "../utils/utils.h"
#include "lidar_config.h"


namespace modules_vins
{

LidarParameters::LidarParameters(){

}


void LidarParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

}


} // namespace modules_vins