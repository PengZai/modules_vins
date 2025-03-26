#include "ros1_visualizer.h"



namespace modules_vins{

    

    ROS1Visualizer::ROS1Visualizer(const std::shared_ptr<SystemConfig> &config, const std::shared_ptr<ros::NodeHandle> &nh):
    config_(config), nh_(nh), it_(*nh)
    {

        for(int i=0; i < config->params_->max_cameras_; i++){
            this->output_image_pub_vector_.push_back(this->it_.advertise(config->camera_config_->params_vector_.at(i)->output_rostopic_, 1));
        }



        this->output_pose_pub_ = this->nh_->advertise<geometry_msgs::PoseStamped>(config->params_->output_pose_rostopic_, 1);
        this->output_trajectory_pub_ = this->nh_->advertise<nav_msgs::Path>(config->params_->output_trajectory_rostopic_, 1);
        this->output_tracked_map_points_pub_ = this->nh_->advertise<sensor_msgs::PointCloud2>(config->params_->output_tracked_map_points_rostopic_, 1);

        // just for test
        this->R_.setIdentity();  // Rotation matrix
        this->t_.setZero();  // Translation vector

        nav_msgs::Path path_msg_;
        

    }

    void ROS1Visualizer::setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh){
        this->nh_ = nh;
    }

    void ROS1Visualizer::publish(const CameraFrame &camera_frame){
        


        if(ros::ok()){
            publish_images(camera_frame);
            // publish_poses(camera_frame);
            publish_trajectory(camera_frame);
        }

 

    }

    void ROS1Visualizer::publish_images(const CameraFrame &camera_frame){

        std_msgs::Header header;

        for(int i=0; i < this->config_->params_->max_cameras_; i++){
            const std::shared_ptr<Image> &img = camera_frame.image_vector_.at(i);
            header.stamp = ros::Time::now();
            header.frame_id = "cam" + img->sensor_id_;
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img->data_).toImageMsg();
            output_image_pub_vector_.at(i).publish(msg);
        
        }


    }


    void ROS1Visualizer::publish_poses(const CameraFrame &camera_frame){

        const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";

        Eigen::Matrix3d rotation = img_0->getRotation();
        Eigen::Vector3d position = img_0->getPosition();

        // Example translation and rotation
        Eigen::Quaterniond q(rotation);

        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = position.z();

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        this->output_pose_pub_.publish(pose_msg);

    }


    void ROS1Visualizer::publish_trajectory(const CameraFrame &camera_frame){

        const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);
        
        path_msg_.header.stamp = ros::Time::now();
        path_msg_.header.frame_id = "map";

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";

        Eigen::Matrix3d rotation = img_0->getRotation();
        Eigen::Vector3d position = img_0->getPosition();

        Eigen::Quaterniond q(rotation);

        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = position.z();

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        this->path_msg_.poses.push_back(pose_msg);

        this->output_trajectory_pub_.publish(this->path_msg_);

    }



    

} //modules_vins