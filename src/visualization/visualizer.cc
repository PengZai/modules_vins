#include "visualizer.h"



namespace modules_vins{


    Visualizer::Visualizer(const std::shared_ptr<SystemConfig> &config, const std::shared_ptr<ros::NodeHandle> &nh):
    config_(config)
    {

        this->opencv_visualizer_ = std::make_shared<OpenCVVisualizer>(config);
        this->ros1_visualizer_ = std::make_shared<ROS1Visualizer>(config, nh);
    
    }

    Visualizer::~Visualizer(){

        cv::destroyAllWindows();

    }



    void Visualizer::publish(const CameraFrame &camera_frame){

        this->opencv_visualizer_->publish(camera_frame);
        this->ros1_visualizer_->publish(camera_frame);
    }

 



} //modules_vins


