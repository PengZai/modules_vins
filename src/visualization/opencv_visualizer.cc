#include "opencv_visualizer.h"


namespace modules_vins
{

    OpenCVVisualizer::OpenCVVisualizer(const std::shared_ptr<SystemConfig> &sys_config):
    sys_config_(sys_config), GreenColor_(cv::Scalar(0,255,0)), RedColor_(cv::Scalar(0,0,255)), BlueColor_(cv::Scalar(255,0,0))
    {

        // for(int i=0; i < config->params_->max_cameras_; i++){
        //     cv::namedWindow("Image"+std::to_string(i), cv::WINDOW_AUTOSIZE);
        // }
    }

    void OpenCVVisualizer::drawTrackingPointPattern(cv::Mat &img, const std::shared_ptr<KeyPoint> &keypoint, const cv::Scalar &color){

        const float r = 5;
        cv::Point2f pt1,pt2;
        
        pt1.x=keypoint->pt2_.x-r;
        pt1.y=keypoint->pt2_.y-r;
        pt2.x=keypoint->pt2_.x+r;
        pt2.y=keypoint->pt2_.y+r;

        cv::rectangle(img, pt1, pt2, color);
        cv::circle(img, keypoint->cv_keypoint_.pt, 2, color, -1);
        
        
    }

    void OpenCVVisualizer::publish_cross_frame(CameraFrame camera_frame){


        std::shared_ptr<Image> img_0;
        cv::Mat img_0_data;

        for(int i=0; i<(int)camera_frame.image_vector_.size(); i++){

            
            const std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);
            cv::Mat img_i_data = img_i->data_.clone();

            if(i==0){
                img_0 = img_i;
                img_0_data = img_i_data;
            }
            
            
            for(const std::shared_ptr<KeyPoint> &keypoint : img_i->keypoint_vector_){

                if(keypoint->match_in_frame_.trainIdx != -1){
    
                    drawTrackingPointPattern(img_i_data, keypoint, this->GreenColor_);
                }
                else{
    
                    drawTrackingPointPattern(img_i_data, keypoint, this->RedColor_);
                }
            }

      
            // cv::imshow("tracking point in frame for image "+std::to_string(i), img_i_data);
            

            if(i>0){

                cv::Mat img_0_matches_in_frame;
                cv::drawMatches(img_i_data, img_0->cv_keypoint_vector_, img_i_data, img_i->cv_keypoint_vector_, img_0->matches_in_frame_, img_0_matches_in_frame,
                    cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
                );

                cv::imshow("matches in frame between img 0 and img " + std::to_string(i), img_0_matches_in_frame);

            }
            
        }



    }

    void OpenCVVisualizer::publish_cross_time(CameraFrame camera_frame){



        const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);
        cv::Mat img_0_data = img_0->data_.clone();

        this->camera_frame_deque_.push_back(camera_frame);
        this->img_deque_.push_back(img_0_data);

  
        for(const std::shared_ptr<KeyPoint> &keypoint : img_0->keypoint_vector_){

            if(keypoint->match_in_time_.trainIdx != -1){

                drawTrackingPointPattern(img_0_data, keypoint, this->GreenColor_);
            }
            else{

                drawTrackingPointPattern(img_0_data, keypoint, this->RedColor_);
            }
            
        }

        cv::imshow("tracking point in time for image 0", img_0_data);


        if(camera_frame_deque_.size()>1){

            CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();
            cv::Mat img_0_data_from_previous_camera_frame = this->img_deque_.front();

            const std::shared_ptr<Image> &img_0_from_previous_camera_frame = previous_camera_frame.image_vector_.at(0);
       

            cv::Mat img_0_matches_in_time;
            cv::drawMatches(img_0_data, img_0->cv_keypoint_vector_, img_0_data_from_previous_camera_frame, img_0_from_previous_camera_frame->cv_keypoint_vector_, img_0->matches_in_time_, img_0_matches_in_time,
                cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
            );

            this->camera_frame_deque_.pop_front();
            this->img_deque_.pop_front();

            cv::imshow("img 0 matches in time", img_0_matches_in_time);

        }

    }   






    void OpenCVVisualizer::publish(const CameraFrame &camera_frame){


        publish_cross_frame(camera_frame);
        publish_cross_time(camera_frame);

        // cv::imshow("test for image "+std::to_string(0), camera_frame.image_vector_.at(0)->data_);

        cv::waitKey(1);



    }
    
} // namespace modules_vins


