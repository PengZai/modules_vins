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

    const double r = 5;
    cv::Point2i pt2i_1,pt2i_2;
    
    pt2i_1.x=keypoint->pt2i_.x-r;
    pt2i_1.y=keypoint->pt2i_.y-r;
    pt2i_2.x=keypoint->pt2i_.x+r;
    pt2i_2.y=keypoint->pt2i_.y+r;

    cv::rectangle(img, pt2i_1, pt2i_2, color);
    cv::circle(img, keypoint->cv_keypoint_.pt, 2, color, -1);
    
    
}

void OpenCVVisualizer::publishMatchingInFrame(CameraFrame camera_frame){


    std::shared_ptr<Image> img_0;
    cv::Mat img_0_data;

    for(int i=0; i<(int)camera_frame.image_vector_.size(); i++){

        
        const std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);
        cv::Mat img_i_data = img_i->color_data_.clone();

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

 
        if(i>0){

            cv::Mat img_0_matches_in_frame;
            cv::drawMatches(img_i_data, img_0->cv_keypoint_vector_, img_i_data, img_i->cv_keypoint_vector_, img_0->matches_in_frame_, img_0_matches_in_frame,
                cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
            );

            cv::imshow("matches in frame between img 0 and img " + std::to_string(i), img_0_matches_in_frame);

        }
        
    }



}

void OpenCVVisualizer::publishMatchingInTime(CameraFrame camera_frame){


    const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->color_data_.clone();

    this->camera_frame_deque_.push_back(camera_frame);
    this->img_deque_.push_back(img_0_color_data);


    for(const std::shared_ptr<KeyPoint> &keypoint : img_0->keypoint_vector_){

        if(keypoint->match_in_time_.trainIdx != -1){

            drawTrackingPointPattern(img_0_color_data, keypoint, this->GreenColor_);
        }
        else{

            drawTrackingPointPattern(img_0_color_data, keypoint, this->RedColor_);
        }
        
    }


    if(camera_frame_deque_.size()>1){

        CameraFrame &previous_camera_frame = this->camera_frame_deque_.front();
        cv::Mat img_0_color_data_from_previous_camera_frame = this->img_deque_.front();

        const std::shared_ptr<Image> &img_0_from_previous_camera_frame = previous_camera_frame.image_vector_.at(0);
    

        cv::Mat img_0_matches_in_time;
        cv::drawMatches(img_0_color_data, img_0->cv_keypoint_vector_, img_0_color_data_from_previous_camera_frame, img_0_from_previous_camera_frame->cv_keypoint_vector_, img_0->matches_in_time_, img_0_matches_in_time,
            cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
        );

        this->camera_frame_deque_.pop_front();
        this->img_deque_.pop_front();

        cv::imshow("img 0 matches in time", img_0_matches_in_time);

    }

}   


void OpenCVVisualizer::publishProjectedMapPoint(const CameraFrame &camera_frame){


    const std::shared_ptr<Image> img_0 = camera_frame.image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->color_data_.clone();

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img_0->sensor_id_)->getCVIntrinsicsMatrix();

    // const std::shared_ptr<Map> &map = camera_frame.getMap();
    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = camera_frame.getMap()->getMapPoints();

    for(const std::pair<const unsigned int, std::shared_ptr<MapPoint>> &item_pair: map_points){
       const std::shared_ptr<MapPoint> &map_point = item_pair.second;

       const Eigen::Vector3d pt3d_in_cam = img_0->T_c_w_ * map_point->pt3d_;
       cv::Point2d reprojected_pixel = camera2pixel(cv::Point3d(pt3d_in_cam.x(), pt3d_in_cam.y(), pt3d_in_cam.z()), cv_K);
       if(img_0->isInImage(reprojected_pixel)){
        cv::circle (img_0_color_data, reprojected_pixel, 5, cv::Scalar (0,255,0), 2);
       }

    }

    cv::imshow("projected map point on img " + std::to_string(img_0->sensor_id_), img_0_color_data);





}


void OpenCVVisualizer::publishDepth(const CameraFrame &camera_frame){


    for(int i=0; i<(int)camera_frame.image_vector_.size(); i++){
        const std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);

        cv::Mat img_i_sensor_detph = img_i->sensor_depth_.clone();
        cv::imshow("sensor depth in frame for image "+std::to_string(i), img_i_sensor_detph);
    }

}



void OpenCVVisualizer::publishTrackingInTime(const CameraFrame &camera_frame){

    const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->color_data_.clone();

    this->camera_frame_deque_.push_back(camera_frame);
    this->img_deque_.push_back(img_0_color_data);


    for(const std::shared_ptr<KeyPoint> &keypoint : img_0->keypoint_vector_){

        if(keypoint->match_in_time_.trainIdx != -1){

            drawTrackingPointPattern(img_0_color_data, keypoint, this->GreenColor_);
        }
        else{

            drawTrackingPointPattern(img_0_color_data, keypoint, this->RedColor_);
        }
        
    }

    cv::imshow("tracking point in time for image 0", img_0_color_data);

}



void OpenCVVisualizer::publish(const CameraFrame &camera_frame){

    if(this->sys_config_->visualizer_config_->opencv_params_->show_matching_in_frame_){
        publishMatchingInFrame(camera_frame);

    }

    if(this->sys_config_->visualizer_config_->opencv_params_->show_matching_in_time_){
        publishMatchingInTime(camera_frame);
    }
    if(this->sys_config_->visualizer_config_->opencv_params_->show_projected_mappoint_){
        publishProjectedMapPoint(camera_frame);
    }

    if(this->sys_config_->visualizer_config_->opencv_params_->show_depth_){

        publishDepth(camera_frame);
    }

    if(this->sys_config_->visualizer_config_->opencv_params_->show_tracking_in_time_){

        publishTrackingInTime(camera_frame);
    }



    cv::waitKey(1);



}
    
} // namespace modules_vins


