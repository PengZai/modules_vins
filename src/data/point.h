#pragma once

#include<opencv2/opencv.hpp>


namespace modules_vins{


class MapPoint{

    public:

    MapPoint();
    MapPoint(const cv::Point3f pt);

    void setPosition(const cv::Point3f pt);

    cv::Point3f pt_;

    static int id_counter_;
    int id_ = -1;





};

class KeyPoint{


    public:
    KeyPoint();
    KeyPoint(const cv::KeyPoint kp);

    void setCVKeyPoint(const cv::KeyPoint &kp);
    void setKeyPointPosition(const double x, const double y);
    void setMatchInTime(const cv::DMatch &match_in_time);
    void setMapPointPtr(const std::shared_ptr<MapPoint> map_point_ptr);

    // void setMatchInFrame(const cv::DMatch &match_in_frame);

    public:
    static int id_counter_;
    int id_ = -1;

    cv::Point2f pt_;
    std::shared_ptr<MapPoint> map_point_ptr_;


    cv::KeyPoint cv_keypoint_;

    // struct cv::DMatch {
    //     int queryIdx;  // Index of keypoint in the first image (query image)
    //     int trainIdx;  // Index of keypoint in the second image (train image)
    //     int imgIdx;    // Index of the image in the train set (useful in multi-image matching)
    //     float distance; // Distance between the descriptors (lower is better)
    // };
    cv::DMatch match_in_time_;
    // cv::DMatch match_in_frame_;

};




} //namespace modules_vins
