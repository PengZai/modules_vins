#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>



namespace modules_vins{


cv::Point2d pixel2norm(const cv::Point2d &pt, const cv::Mat &K );
cv::Point3d pixel2camera(const cv::Point2d &pt, double depth, cv::Mat &K);
cv::Point2d camera2pixel(const cv::Point3d &pt3d, cv::Mat &K);

} //namespace modules_vins



