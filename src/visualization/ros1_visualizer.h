#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

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


    public:
    std::shared_ptr<SystemConfig> config_;
    std::shared_ptr<ros::NodeHandle> nh_;
    image_transport::ImageTransport it_;
    std::vector<image_transport::Publisher> output_image_pub_vector_;

};


    
} // namespace modules_vins






