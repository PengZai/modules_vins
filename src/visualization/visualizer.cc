#include "visualizer.h"



namespace modules_vins{





Visualizer::Visualizer(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh):
sys_config_(sys_config)
{
    if(this->sys_config_->visualizer_config_->params_->use_opencv_vis_){
        this->opencv_visualizer_ = std::make_shared<OpenCVVisualizer>(sys_config);

    }

    if(this->sys_config_->visualizer_config_->params_->use_pangolin_vis_){
        this->pangolin_visualizer_ = std::make_shared<PangolinVisualizer>(sys_config);

    }


    if(this->sys_config_->visualizer_config_->params_->use_rviz_vis_){
        this->ros1_visualizer_ = std::make_shared<ROS1Visualizer>(sys_config, nh);

    }



    

}

Visualizer::~Visualizer(){

    cv::destroyAllWindows();

}


std::shared_ptr<OpenCVVisualizer> & Visualizer::getOpenCVVisualizer(){
    return this->opencv_visualizer_;
}


void Visualizer::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;

    if(this->sys_config_->visualizer_config_->params_->use_opencv_vis_){
        this->opencv_visualizer_->setMap(map);
    }

    if(this->sys_config_->visualizer_config_->params_->use_rviz_vis_){
        this->ros1_visualizer_->setMap(map);
    }

    if(this->sys_config_->visualizer_config_->params_->use_pangolin_vis_){
        this->pangolin_visualizer_->setMap(map);
    }

}



void Visualizer::publish(const std::shared_ptr<CameraFrame> &camera_frame, const std::shared_ptr<State> &state){

    
    if(this->sys_config_->visualizer_config_->params_->use_opencv_vis_){
        this->opencv_visualizer_->publish(camera_frame);
    }

    if(this->sys_config_->visualizer_config_->params_->use_pangolin_vis_){
        this->pangolin_visualizer_->publish(state);
    }

    if(this->sys_config_->visualizer_config_->params_->use_rviz_vis_){
        this->ros1_visualizer_->publish(camera_frame, state);
    }

    
}

 



} //modules_vins


