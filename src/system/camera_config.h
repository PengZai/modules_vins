#pragma once


#include <memory>
#include <Eigen/Dense>
#include "config.h"

#include "../utils/utils.h"


namespace modules_vins
{

class CameraParameters : public Parameters
{

    public:

        CameraParameters();

        void loadFromNode(const std::shared_ptr<cv::FileNode> &node);
        Eigen::Matrix3d getIntrinsicsMatrix();


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

        void calculateExtrinsicsAndProjectionMatrixBetweenCameras();

        Eigen::Matrix4d getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_0, const unsigned int sensor_id_1);
        Eigen::Matrix<double, 3, 4> getProjectionMatrixBetweenCamerasBySensorID(Eigen::Matrix4d T_cam_i_world, const unsigned int sensor_id_i, const unsigned int sensor_id_j);

    public:
        std::vector<std::shared_ptr<CameraParameters>> params_vector_;

    protected:

        std::unordered_map<std::pair<unsigned int, unsigned int>, Eigen::Matrix4d, pair_hash> map_extrinsics_between_cameras_;

}; 





} // namespace modules_vins