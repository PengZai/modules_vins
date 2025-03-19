#pragma once

#include<opencv2/opencv.hpp>

namespace modules_vins{


class KeyPoint{


    public:
    KeyPoint();
    KeyPoint(const cv::KeyPoint &kp);

    void setId(int id);
    void setCVKeyPoint(const cv::KeyPoint &kp);
    void setKeyPointPosition(const double x, const double y);
    void setMatchInTime(const cv::DMatch &match_in_time);
    // void setMatchInFrame(const cv::DMatch &match_in_frame);

    public:
    int id_ = -1;
    float x_;
    float y_;

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
