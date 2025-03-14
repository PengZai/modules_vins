#include "opencv_visualizer.h"


namespace modules_vins
{

    OpenCVVisualizer::OpenCVVisualizer(const std::shared_ptr<SystemConfig> &config){


        this->config_ = config;
        for(int i=0; i < config->params_->max_cameras_; i++){
            cv::namedWindow("Image"+std::to_string(i), cv::WINDOW_AUTOSIZE);
        }
    }

    void OpenCVVisualizer::publish(const CameraFrame &camera_frame){

        for(int i=0; i < this->config_->params_->max_cameras_; i++){
            const Image &img = camera_frame.image_vector_.at(i);
            cv::imshow("Image"+std::to_string(i), img.data_);
        }
        cv::waitKey(1);



    }
    
} // namespace modules_vins


