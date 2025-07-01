#pragma once
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>           // For loading/saving PCD files
#include <pcl_conversions/pcl_conversions.h> // for pcl::toROSMsg


#include "../system/state.h"
#include "../system/system_config.h"
#include "../data/camera.h"
#include "../log/logging.h"



namespace modules_vins
{



class ROS1Visualizer{


    public:

    ROS1Visualizer(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh);

    void setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh);
    void setMap(const std::shared_ptr<Map> &map);
    void publish(const std::shared_ptr<CameraFrame> &camera_frame, const std::shared_ptr<State> &state);
    void publishTF();
    void publishImages(const std::shared_ptr<CameraFrame> &camera_frame);
    void constructPoseMsg(const Sophus::SE3<double> &pose, geometry_msgs::PoseStamped &pose_msg);
    void constructPoseMsg(const Sophus::SE3<double> &pose, geometry_msgs::Pose &pose_msg);
    void publishPoses(const std::shared_ptr<State> &state);
    void publishKeyPoses(const std::shared_ptr<State> &state);
    void publishTrajectories(const std::shared_ptr<State> &state);
    void publishTrajectory(const std::map<double, Sophus::SE3<double>> &timestamp_T_c_w_map, nav_msgs::Path &path_msgs, ros::Publisher output_trajectory_pub);
    void publishGTTrajectory(const std::map<double, Sophus::SE3<double>> &timestamp_T_c_w_map, nav_msgs::Path &path_msgs, ros::Publisher output_trajectory_pub); 
    void publishMapPoint(const std::shared_ptr<State> &state);


    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    std::shared_ptr<ros::NodeHandle> nh_;
    image_transport::ImageTransport it_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    std::vector<image_transport::Publisher> output_image_pub_vector_;
    ros::Publisher output_pose_pub_;
    ros::Publisher output_key_frame_poses_pub_;
    std::vector<ros::Publisher> output_comparison_pose_pub_vector_;
    std::vector<ros::Publisher> output_comparison_trajectory_pub_vector_;

    ros::Publisher output_tracked_map_points_pub_;
    ros::Publisher output_trajectory_pub_;
    ros::Publisher output_point_cloud_pub_;
    ros::Rate ros_rate_;


    std::string pose_frame_id_;


    // just for test
    Eigen::Matrix3d R_;  // Rotation matrix
    Eigen::Vector3d t_;  // Translation vector



};


    
} // namespace modules_vins






