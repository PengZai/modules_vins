#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <cmath>


#include "point.h"



namespace modules_vins {


struct SegmentOutput {
    int class_id_; // class id
    float confidence_; // confidence score
    cv::Rect2i box_; // bounding box
    cv::Mat boxMask_; // mask in bounding box
};

struct BoxOutput {
    int class_id_; // class id
    float confidence_; // confidence score
    cv::Rect2f box_; // bounding box
};

class Image{

    public:

        Image(double timestamp, int sensor_id, cv::Mat data);
        std::vector<Eigen::Vector3d> getMapPoints() const;

        void setSensorDepth(const cv::Mat &sensor_depth);
        void setTcw(const Sophus::SE3<double> T_c_w);
        void setTcw(const Eigen::Matrix3d &rotation, Eigen::Vector3d position);

        double getPointDepthFromSensor(const cv::Point2f &pt);
        void cleanTrackInTimeRelationship();
        void cleanTrackInFrameRelationship();
        void cleanFeaturePoints();

        Eigen::Matrix3d getRotation();
        Eigen::Vector3d getPosition();
        
        bool isInImage(const cv::Point2d &pixel);
        void setKeyPoints(std::vector<cv::KeyPoint> &cv_key_points);
        void setKeyPoints(std::vector<cv::KeyPoint> &cv_key_points, const cv::Mat &descriptors);
        void appendKeyPoints(std::vector<cv::KeyPoint> &cv_key_points);
        void appendKeyPoints(std::vector<cv::KeyPoint> &cv_key_points, const cv::Mat &descriptors);
        void getCVKeyPoints(std::vector<cv::KeyPoint> &cv_key_points);
        void getDescripots(cv::Mat &descriptors);


    public:
        static int id_counter_;
        int id_;
        double timestamp_;

        //which camera this image belong to
        int sensor_id_; 
        cv::Mat color_data_;
        cv::Mat gray_data_;
        cv::Mat normalize_gray_data_;

        cv::Mat depth_;
        cv::Mat learned_depth_;
        cv::Mat stereo_depth_;
        cv::Mat learned_stereo_depth_;
        cv::Mat sensor_depth_;
        // std::vector<cv::KeyPoint> cv_keypoint_vector_;

        std::vector<std::shared_ptr<KeyPoint>> keypoint_vector_;

        // for yolo object detection
        std::vector<BoxOutput> box_outputs_;
        // for yolo semantic segmentation
        std::vector<SegmentOutput> segment_outputs_;
        
    
        // matches for previous
        std::vector<cv::DMatch> matches_in_time_;

        // matches in frame
        std::vector<cv::DMatch> matches_in_frame_;

        // Eigen::Matrix<double, 4, 4> T_c_w_;
        Sophus::SE3<double> T_c_w_;



        friend std::ostream& operator<<(std::ostream& os, const Image& img);

        

};




class CameraFrame {

    public:
    CameraFrame();
    CameraFrame(const std::vector<std::shared_ptr<Image>> image_vector);
    
    void cleanTrackInTimeRelationship();
    void cleanTrackInFrameRelationship();
    void setTrackInTimeRelationship(const std::vector<cv::DMatch> &matches);
    void setTrackInFrameRelationship(const std::vector<cv::DMatch> &matches);
    void propogateMappointWitchMatchInTimeRelationship();

    void cleanFeaturePoints();

    // CameraFrame(const CameraFrame &camera_frame);

    std::vector<std::shared_ptr<Image>> image_vector_;
    static int id_counter_;
    int id_;
    std::shared_ptr<CameraFrame> ref_camera_frame_;


    enum Status{
        NOT_INITIALIZED=-1,
        NORMAL=0,
        FAIL
    };

    bool is_key_camera_frame_;

    Status status_;



    // Sophus::SE3<double> Tcw;




};


// class KeyCameraFrame : public CameraFrame {

//     public:

//     KeyCameraFrame();
//     KeyCameraFrame(const std::vector<std::shared_ptr<Image>> image_vector);

//     static int id_counter_;
//     std::shared_ptr<KeyCameraFrame> ref_key_camera_frame_;



// };


}



