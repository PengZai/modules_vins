#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


#include "../../system/system_config.h"
#include "../../data/point.h"
#include "../../data/camera.h"
#include "../../log/logging.h"


namespace modules_vins
{


class TwoViewReconstructor{



    public:
    TwoViewReconstructor(const std::shared_ptr<SystemConfig> &sys_config);    
    void reconstruct(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1);
    cv::Point2f pixel2norm (const cv::Point2f &pt, const cv::Mat &K );  
    void checkTriangulatedPointsWithReprojection(const cv::Point2f &pt2, const cv::Point3f &pt3, const cv::Mat &T, const cv::Mat &K);
    
    std::shared_ptr<SystemConfig> sys_config_;

};

    
} // namespace modules_vins








