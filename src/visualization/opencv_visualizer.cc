#include "opencv_visualizer.h"


namespace modules_vins
{

    OpenCVVisualizer::OpenCVVisualizer(const std::shared_ptr<SystemConfig> &config):
    config_(config), GreenColor_(cv::Scalar(0,255,0)), RedColor_(cv::Scalar(0,0,255)), BlueColor_(cv::Scalar(255,0,0))
    {

        // for(int i=0; i < config->params_->max_cameras_; i++){
        //     cv::namedWindow("Image"+std::to_string(i), cv::WINDOW_AUTOSIZE);
        // }
    }

    void OpenCVVisualizer::publish(const CameraFrame &camera_frame){

        this->camera_frame_deque_.push_back(camera_frame);
        
        cv::Point2f pt1,pt2;

        for(int i=0; i < this->config_->params_->max_cameras_; i++){
            const Image &img = camera_frame.image_vector_.at(i);

            // we only draw keypoint on camera0
            if(img.sensor_id_ == 0){

                

                for(const KeyPoint &keypoint : img.keypoint_vector_){

                    const float r = 5;
                    pt1.x=keypoint.x_-r;
                    pt1.y=keypoint.y_-r;
                    pt2.x=keypoint.x_+r;
                    pt2.y=keypoint.y_+r;
    
                    if(keypoint.match_in_time_.trainIdx != -1){
                        cv::rectangle(img.data_,pt1,pt2,this->GreenColor_);
                        cv::circle(img.data_,keypoint.cv_keypoint_.pt,2,this->GreenColor_,-1);
                    }
                    else{
                        cv::rectangle(img.data_,pt1,pt2,this->RedColor_);
                        cv::circle(img.data_,keypoint.cv_keypoint_.pt,2,this->RedColor_,-1);
                    }
                    
                }

                if(camera_frame_deque_.size()>1){

                    CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();
                    const Image &img_from_previous_camera_frame = previous_camera_frame.image_vector_.at(0);
                    cv::Mat img_matches;
    
                    cv::drawMatches(img.data_, img.cv_keypoint_vector_, img_from_previous_camera_frame.data_, img_from_previous_camera_frame.cv_keypoint_vector_, img.matches_in_time_, img_matches,
                        cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
                    );

                    this->camera_frame_deque_.pop_front();

                    cv::imshow("Matches", img_matches);

                }

                cv::imshow("Image"+std::to_string(i), img.data_);

            }
            
        }



   

        cv::waitKey(1);



    }
    
} // namespace modules_vins


