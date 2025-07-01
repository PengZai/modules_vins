#pragma once


#include <Eigen/Core>               
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "../data/camera.h"
#include "../system/system_config.h"
#include "../optimization/bundle_adjustment.h"


namespace modules_vins
{




class PoseEstimator{

    public:

    PoseEstimator(const std::shared_ptr<SystemConfig> &sys_config);
    bool checkEstimatedPose(const Sophus::SE3<double> &estimated_T,const Sophus::SE3<double> &initial_guess, const int num_inliers);
    int epipolarGeometryEstimator(const std::vector<cv::Point2d> &ref_pt2ds, 
        const std::vector<cv::Point2d> &pt2ds, 
        cv::Mat &cv_R, 
        cv::Mat &translation_vec,
        const cv::Mat &cv_K);
    int PnpEstimator(
        const std::vector<cv::Point3d> &pt3ds, 
        const std::vector<cv::Point2d> &pt2ds, 
        Sophus::SE3<double> &estimated_T
    );

    int PnpEstimator(
        const std::vector<cv::Point3d> &pt3ds, 
        const std::vector<cv::Point2d> &pt2ds, 
        Sophus::SE3<double> &estimated_T,
        const cv::Mat K,
        const cv::Mat cv_distortion_coeffs);

    std::shared_ptr<SystemConfig> sys_config_;
   
    bool pipeline(const std::shared_ptr<CameraFrame> &camera_frame, int &num_inlier, double maximum_motion_norm);


};


    
} // namespace modules_vins


