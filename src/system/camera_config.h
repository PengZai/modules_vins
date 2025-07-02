#pragma once


#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "../utils/utils.h"


namespace modules_vins
{

class CameraParameters : public Parameters
{

    public:

        CameraParameters();

        void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;
        const Eigen::Matrix3d getIntrinsicsMatrix();
        const cv::Mat getCVIntrinsicsMatrix();
        const Eigen::VectorXd getDistortionCoeffs();
        const cv::Mat getCVDistortionCoeffs();


    public:


        Eigen::VectorXd resolution_;
        Eigen::VectorXd distortion_coeffs_;
        Eigen::VectorXd intrinsics_;

        std::string rgb_rostopic_;

        bool use_sensor_depth_;
        std::string sensor_depth_rostopic_;

        bool use_learned_depth_;
        std::string model_name_learned_depth_;
        std::string output_learned_depth_rostopic_;

        bool use_stereo_matching_;
        std::string stereo_matching_rostopic_;

        bool use_learned_stereo_matching_;
        std::string model_name_learned_stereo_matching_;
        std::string learned_stereo_matching_rostopic_;

        bool use_learned_object_detection_;
        std::string model_name_learned_object_detection_;
        std::string learned_object_detection_rostopic_;

        bool use_learned_semantic_segmentation_;
        std::string model_name_learned_semantic_segmentation_;
        std::string learned_semantic_segmentation_rostopic_;
     
        std::string output_rostopic_;
        std::string camera_model_;
        std::string distortion_model_;
        

};

class CameraConfig : public Config
{
    public:


        void calculateExtrinsicsAndProjectionMatrixBetweenCameras();

        Eigen::Matrix<double, 4, 4> getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_0, const unsigned int sensor_id_1);
        // Eigen::Matrix<double, 3, 4> getProjectionMatrixBetweenCamerasBySensorID(Eigen::Matrix4d T_cam_i_world, const unsigned int sensor_id_i, const unsigned int sensor_id_j);

    public:

    protected:


}; 





} // namespace modules_vins