#include "visualizer.h"



namespace modules_vins{


    Visualizer::Visualizer(const std::shared_ptr<SystemConfig> &config, const std::shared_ptr<ros::NodeHandle> &nh):
    config_(config)
    {
        if(this->config_->params_->use_opencv_vis_){
            this->opencv_visualizer_ = std::make_shared<OpenCVVisualizer>(config);
        }
        if(this->config_->params_->use_rviz_vis_){
            this->ros1_visualizer_ = std::make_shared<ROS1Visualizer>(config, nh);
        }
    
    }

    Visualizer::~Visualizer(){

        cv::destroyAllWindows();

    }



    void Visualizer::publish(const CameraFrame &camera_frame){


        if(this->config_->params_->use_opencv_vis_){
            this->opencv_visualizer_->publish(camera_frame);
        }
        if(this->config_->params_->use_rviz_vis_){
            this->ros1_visualizer_->publish(camera_frame);
        }
    }

 



} //modules_vins


