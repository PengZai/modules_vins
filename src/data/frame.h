#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include "camera.h"



namespace modules_vins {



class Frame {

    public:
    Frame();
    
    void setImages(const std::vector<std::shared_ptr<Image>> &image_vector);

    void cleanTrackInTimeRelationship();
    void cleanTrackInFrameRelationship();
    void setTrackInTimeRelationship(const std::vector<cv::DMatch> &matches);
    void setTrackInFrameRelationship(const std::vector<cv::DMatch> &matches);
    // void setTcwWithCamera0(const Sophus::SE3d &Tc0w);
    void setCamera0VelocityWithCamera0Tcw();
    void initializeTbwWithVelocity();
    void propogateMappointWitchMatchInTimeRelationship();

    void setTbw(const Sophus::SE3<double> T_b_w);
    void setTbw(const Eigen::Matrix3d &rotation, Eigen::Vector3d position);
    void setVelocityTbw(const Sophus::Vector6d Velocity_T_b_w);

    void cleanFeaturePoints();

    // Frame(const Frame &frame);

    static int id_counter_;
    int id_;
    double timestamp_;


    std::vector<std::shared_ptr<Image>> image_vector_;
    

    std::shared_ptr<Frame> ref_frame_;


    enum Status{
        NOT_INITIALIZED=-1,
        NORMAL=0,
        FAIL
    };


    bool use_comparison_pose_for_pose_estimation_;
    size_t comparison_pose_idx_for_pose_estimation_; 
    
    bool is_key_frame_;

    Status status_;

    Sophus::SE3<double> T_b_w_;
    Sophus::Vector6d Velocity_T_b_w_;



    // Sophus::SE3<double> Tcw;




};

} //modules_vins