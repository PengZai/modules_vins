#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>


namespace modules_vins {


class Image{

    public:

    Image(double timestamp, int sensor_id, cv::Mat data);

    public:
        double timestamp_;
        int sensor_id_;
        cv::Mat data_;
        cv::Mat gray_data_;
        std::vector<cv::KeyPoint> keypoint_vector_;
        cv::Mat descriptors_;

        // The key int represents a index of keypoint in this image.
        // The value std::vector<std::pair<int, int>> represents a list of associated keypoints in different images.
        std::map<int, std::vector<std::pair<int, int>>> keypoint_graph;


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

    std::vector<Image> image_vector_;
    int frame_id_;

    // Sophus::SE3<float> Tcw;

    
    // The key std::pair<int, int> represents a keypoint in (image_index, keypoint_index) format.
    // The value std::vector<std::pair<int, int>> represents a list of associated keypoints in different images.
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> keypoint_graph;



};





}



