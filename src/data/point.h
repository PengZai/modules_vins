#pragma once

#include<opencv2/opencv.hpp>
#include<memory>

namespace modules_vins{


class MapPoint{

    public:

    MapPoint();
    MapPoint(const cv::Point3f pt);

    void setPosition(const cv::Point3f pt);

    int id_ = -1;
    cv::Point3f pt_;

    static int id_counter_;
    





};

class KeyPoint{


    public:
    KeyPoint();
    KeyPoint(const cv::KeyPoint kp);

    void setCVKeyPoint(const cv::KeyPoint &kp);
    void setKeyPointPosition(const double x, const double y);
    void setMatchInTime(const cv::DMatch &match_in_time);
    void setMatchInFrame(const cv::DMatch &match_in_frame);

    void setMapPointPtr(const std::shared_ptr<MapPoint> &map_point_ptr);


    void setNextKeyPointInTime(const std::shared_ptr<KeyPoint> &next_keypoint_in_time);
    void setPrevKeyPointInTime(const std::shared_ptr<KeyPoint> &prev_keypoint_in_time);

    // void setMatchInFrame(const cv::DMatch &match_in_frame);

    public:
    static int id_counter_;
    int id_ = -1;

    cv::Point2f pt_;
    cv::KeyPoint cv_keypoint_;

    std::shared_ptr<MapPoint> map_point_ptr_;


    // struct cv::DMatch {
    //     int queryIdx;  // Index of keypoint in the first image (query image)
    //     int trainIdx;  // Index of keypoint in the second image (train image)
    //     int imgIdx;    // Index of the image in the train set (useful in multi-image matching)
    //     float distance; // Distance between the descriptors (lower is better)
    // };
    cv::DMatch match_in_time_;
    cv::DMatch match_in_frame_;

    std::shared_ptr<KeyPoint> next_keypoint_in_time_;
    std::shared_ptr<KeyPoint> prev_keypoint_in_time_;

    // std::shared_ptr<KeyPoint> next_keypoint_in_frame_;

    protected:
    void setMapPointPtrForward(const std::shared_ptr<MapPoint> &map_point_ptr);
    void setMapPointPtrBackward(const std::shared_ptr<MapPoint> &map_point_ptr);


};




} //namespace modules_vins
