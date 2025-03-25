#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>


#include "../log/logging.h"
#include "../system/system_config.h"
#include "../data/camera.h"



namespace modules_vins
{



class ROS1Visualizer{


    public:

    ROS1Visualizer(const std::shared_ptr<SystemConfig> &config, const std::shared_ptr<ros::NodeHandle> &nh);

    void setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh);
    void publish(const CameraFrame &camera_frame);
    void publish_images(const CameraFrame &camera_frame);
    void publish_poses(const CameraFrame &camera_frame);
    void publish_trajectory(const CameraFrame &camera_frame);


    protected:
    std::shared_ptr<SystemConfig> config_;
    std::shared_ptr<ros::NodeHandle> nh_;
    image_transport::ImageTransport it_;
    std::vector<image_transport::Publisher> output_image_pub_vector_;
    ros::Publisher output_pose_pub_;
    ros::Publisher output_tracked_map_points_pub_;
    ros::Publisher output_trajectory_pub_;

    nav_msgs::Path path_msg_;

    // just for test
    Eigen::Matrix3d R_;  // Rotation matrix
    Eigen::Vector3d t_;  // Translation vector

};


    
} // namespace modules_vins






