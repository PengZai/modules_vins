#include "ros1_visualizer.h"



namespace modules_vins{

    

    ROS1Visualizer::ROS1Visualizer(const std::shared_ptr<SystemConfig> &config, const std::shared_ptr<ros::NodeHandle> &nh):
    config_(config), nh_(nh), it_(*nh)
    {

        for(int i=0; i < config->params_->max_cameras_; i++){
            this->output_image_pub_vector_.push_back(it_.advertise(config->camera_config_->params_vector_.at(i)->output_rostopic_, 1));
        }


    }

    void ROS1Visualizer::setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh){
        this->nh_ = nh;
    }

    void ROS1Visualizer::publish(const CameraFrame &camera_frame){
        
        std_msgs::Header header;

        for(int i=0; i < this->config_->params_->max_cameras_; i++){
            const Image &img = camera_frame.image_vector_.at(i);
            header.stamp = ros::Time::now();
            header.frame_id = "cam" + img.sensor_id_;
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img.data_).toImageMsg();
            output_image_pub_vector_.at(i).publish(msg);
        
        }

        

    }

    

} //modules_vins