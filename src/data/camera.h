#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "point.h"

namespace modules_vins {


class Image{

    public:

    Image(double timestamp, int sensor_id, cv::Mat data);

    public:
        double timestamp_;

        static int image_id_counter_;
        int image_id_;

        //which camera this image belong to
        int sensor_id_; 
        cv::Mat data_;
        cv::Mat gray_data_;
        std::vector<cv::KeyPoint> cv_keypoint_vector_;
        std::vector<KeyPoint> keypoint_vector_;
        cv::Mat descriptors_;
        
    
        // matches for previous
        std::vector<cv::DMatch> matches_in_time_;

        // matches in frame
        std::vector<cv::DMatch> matches_in_frame_;




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
    CameraFrame(const std::vector<Image> image_vector);
    // CameraFrame(const CameraFrame &camera_frame);

    std::vector<Image> image_vector_;
    static int frame_id_counter_;
    int frame_id_;

    // Sophus::SE3<float> Tcw;




};





}



