#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>




namespace modules_vins{


cv::Point2d pixel2norm(const cv::Point2d &pt, const cv::Mat &K );
cv::Point3d pixel2camera(const cv::Point2d &pt, const double depth, const cv::Mat &K);
Eigen::Vector3d pixel2camera(const Eigen::Vector2d &pt2d, const double depth, const Eigen::Matrix<double, 3,3> &K);
cv::Point2d camera2pixel(const cv::Point3d &pt3d, const cv::Mat &K);
Eigen::Vector2d camera2pixel(const Eigen::Vector3d &pt3d, const Eigen::Matrix<double, 3, 3> &K);

} //namespace modules_vins



