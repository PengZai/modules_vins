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

        cv::Scalar matchedColor(0,255,0);
        cv::Scalar dismatchedColor(255,0,0);

        const float r = 5;
        cv::Point2f pt1,pt2;

        for(int i=0; i < this->config_->params_->max_cameras_; i++){
            const Image &img = camera_frame.image_vector_.at(i);
            for(const cv::KeyPoint &keypoint : img.keypoint_vector_){
                pt1.x=keypoint.pt.x-r;
                pt1.y=keypoint.pt.y-r;
                pt2.x=keypoint.pt.x+r;
                pt2.y=keypoint.pt.y+r;
                cv::rectangle(img.data_,pt1,pt2,matchedColor);
                cv::circle(img.data_,keypoint.pt,2,matchedColor,-1);
            }
            
            cv::imshow("Image"+std::to_string(i), img.data_);
        }

        // cv::Mat img_matches;
        // const Image &img0 = camera_frame.image_vector_.at(0);
        // const Image &img1 = camera_frame.image_vector_.at(0);

        // cv::drawMatches(img0.data_, img0.keypoint_vector_, img1.data_, img1.keypoint_vector_, img0.matches_in_frame, img_matches);
        // cv::imshow("Matches", img_matches);

        cv::waitKey(1);



    }
    
} // namespace modules_vins


