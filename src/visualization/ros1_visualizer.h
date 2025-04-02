#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>           // For loading/saving PCD files
#include <pcl_conversions/pcl_conversions.h> // for pcl::toROSMsg


#include "../log/logging.h"
#include "../system/system_config.h"
#include "../data/camera.h"



namespace modules_vins
{



class ROS1Visualizer{


    public:

    ROS1Visualizer(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh);

    void setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh);
    void publish(const CameraFrame &camera_frame);
    void publishImages(const CameraFrame &camera_frame);
    void publishPoses(const CameraFrame &camera_frame);
    void publishTrajectory(const CameraFrame &camera_frame);
    void publishMapPoint(const CameraFrame &camera_frame);


    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<ros::NodeHandle> nh_;
    image_transport::ImageTransport it_;
    std::vector<image_transport::Publisher> output_image_pub_vector_;
    ros::Publisher output_pose_pub_;
    ros::Publisher output_tracked_map_points_pub_;
    ros::Publisher output_trajectory_pub_;
    ros::Publisher output_point_cloud_pub_;
    ros::Rate ros_rate_;

    nav_msgs::Path path_msg_;


    // just for test
    Eigen::Matrix3d R_;  // Rotation matrix
    Eigen::Vector3d t_;  // Translation vector

};


    
} // namespace modules_vins






