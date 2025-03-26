#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <Eigen/Dense>

#include "point.h"
#include "map.h"


namespace modules_vins {

class Map;

class Image{

    public:

        Image(double timestamp, int sensor_id, cv::Mat data);
        std::vector<cv::Point3f> getMapPoints() const;
        void initPose();

        void setTcw(const Eigen::Matrix3d &rotation, Eigen::Vector3d position);

        Eigen::Matrix3d getRotation();
        Eigen::Vector3d getPosition();

    public:
        static int id_counter_;
        int id_;
        double timestamp_;

        //which camera this image belong to
        int sensor_id_; 
        cv::Mat data_;
        cv::Mat gray_data_;
        std::vector<cv::KeyPoint> cv_keypoint_vector_;
        cv::Mat descriptors_;

        std::vector<std::shared_ptr<KeyPoint>> keypoint_vector_;
        
    
        // matches for previous
        std::vector<cv::DMatch> matches_in_time_;

        // matches in frame
        std::vector<cv::DMatch> matches_in_frame_;

        Eigen::Matrix<double, 4, 4> T_c_w_;



        friend std::ostream& operator<<(std::ostream& os, const Image& img);

        

};

// class CameraData{


//     public:

//         CameraData();
//         CameraData(const std::vector<double> &timestamp_vector, const std::vector<int> &sensor_id_vector, const std::vector<cv::Mat> &image_vector);

//         std::vector<Image> image_vector_;

      


// };


class CameraFrame {

    public:
    CameraFrame();
    CameraFrame(const std::vector<std::shared_ptr<Image>> image_vector);
    
    void setMap(const std::shared_ptr<Map> &map);
    // CameraFrame(const CameraFrame &camera_frame);

    std::vector<std::shared_ptr<Image>> image_vector_;
    static int id_counter_;
    int id_;


    protected:
    std::shared_ptr<Map> map_;

    // Sophus::SE3<float> Tcw;




};





}



