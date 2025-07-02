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

        void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;

    public:

        

};

class LidarConfig : public Config
{
    public:



    protected:


}; 





} // namespace modules_vins