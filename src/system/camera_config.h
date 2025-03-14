#pragma once


#include <memory>
#include "config.h"

namespace modules_vins
{

class CameraParameters : public Parameters
{

    public:

        CameraParameters();
        void loadFromNode(const std::shared_ptr<cv::FileNode> &node);

    public:
        Eigen::Matrix4d T_imu_cam_;
        Eigen::VectorXd resolution_;
        Eigen::VectorXd distortion_coeffs_;
        Eigen::VectorXd intrinsics_;


        std::string rostopic_;
        std::string output_rostopic_;
        std::string camera_model_;
        std::string distortion_model_;



};

class CameraConfig : public Config
{
    public:

        void loadConfigFromPath(const std::string &config_path) override;

    public:
        std::vector<std::shared_ptr<CameraParameters>> params_vector_;

}; 





} // namespace modules_vins