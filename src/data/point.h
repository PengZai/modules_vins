#pragma once

#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<memory>


#include "../log/logging.h"

namespace modules_vins{


class MapPoint{

    public:

    MapPoint();
    MapPoint(const Eigen::Vector3d &pt3d);

    void setPosition(const Eigen::Vector3d &pt3d);
    void setPosition(const double x, const double y, const double z);
    void setColor(const Eigen::Vector3i &bgr);
    void setColor(const int r, const int g, const int b);


    int id_ = -1;
    static int id_counter_;
    double timestamp_;

    

    Eigen::Vector3d pt3d_; // x y z
    Eigen::Vector3i bgr_; // x y z




};

class KeyPoint{


    public:
    KeyPoint();
    KeyPoint(const cv::KeyPoint kp);

    void setCVKeyPoint(const cv::KeyPoint &kp);
    void set2DKeyPoint(const cv::Point2i &pt2i);
    void set2DKeyPoint(const int x, const int y);
    void set3DKeyPoint(const cv::Point3d &pt3d);
    void set3DKeyPoint(const double x, const double y, const double z);
    void setMatchInTime(const cv::DMatch &match_in_time);
    void setMatchInFrame(const cv::DMatch &match_in_frame);

    void setMapPointPtr(const std::shared_ptr<MapPoint> &map_point_ptr);
    void propagateMapPointPtr();

    void setNextKeyPointInTime(const std::shared_ptr<KeyPoint> &next_keypoint_in_time);
    void setPrevKeyPointInTime(const std::shared_ptr<KeyPoint> &prev_keypoint_in_time);
    void setLeftKeyPointInFrame(const std::shared_ptr<KeyPoint> &left_keypoint_in_frame);
    void setRightKeyPointInFrame(const std::shared_ptr<KeyPoint> &right_keypoint_in_frame);

    void cleanTrackInTimeRelationship();

    // void setMatchInFrame(const cv::DMatch &match_in_frame);

    public:
    static int id_counter_;
    int id_ = -1;
    double timestamp_;

    
    cv::Point2i pt2i_; // keypoint in pixel plane
    cv::Point3d pt3d_; // 3d keypoint in camera coordinate

    cv::KeyPoint cv_keypoint_;

    std::shared_ptr<MapPoint> map_point_ptr_;


    // struct cv::DMatch {
    //     int queryIdx;  // Index of keypoint in the first image (query image)
    //     int trainIdx;  // Index of keypoint in the second image (train image)
    //     int imgIdx;    // Index of the image in the train set (useful in multi-image matching)
    //     double distance; // Distance between the descriptors (lower is better)
    // };
    cv::DMatch match_in_time_;
    cv::DMatch match_in_frame_;

    std::shared_ptr<KeyPoint> next_keypoint_in_time_;
    std::shared_ptr<KeyPoint> prev_keypoint_in_time_;
    std::shared_ptr<KeyPoint> left_keypoint_in_frame_;
    std::shared_ptr<KeyPoint> right_keypoint_in_frame_;

    // std::shared_ptr<KeyPoint> next_keypoint_in_frame_;

    protected:
    void setMapPointPtrForward(const std::shared_ptr<MapPoint> &map_point_ptr);
    void setMapPointPtrBackward(const std::shared_ptr<MapPoint> &map_point_ptr);
    void setMapPointPtrLeftForward(const std::shared_ptr<MapPoint> &map_point_ptr);
    void setMapPointPtrRightForward(const std::shared_ptr<MapPoint> &map_point_ptr);


};




} //namespace modules_vins
