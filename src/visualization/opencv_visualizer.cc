#include "opencv_visualizer.h"



namespace modules_vins
{

OpenCVVisualizer::OpenCVVisualizer(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config), GreenColor_(cv::Scalar(0,255,0)), RedColor_(cv::Scalar(0,0,255)), BlueColor_(cv::Scalar(255,0,0))
{

    // Note that in this example the classes are hard-coded
    this->classes_ = {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
        "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
        "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife",
        "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
        "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
}


void OpenCVVisualizer::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;
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

void OpenCVVisualizer::publishMatchingInFrame(const std::shared_ptr<CameraFrame> &camera_frame){


    std::shared_ptr<Image> img_0;
    cv::Mat img_0_data;

    for(int i=0; i<(int)camera_frame->image_vector_.size(); i++){

        
        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);
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

void OpenCVVisualizer::publishMatchingInTime(const std::shared_ptr<CameraFrame> &camera_frame){


    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->color_data_.clone();


    for(const std::shared_ptr<KeyPoint> &keypoint : img_0->keypoint_vector_){

        if(keypoint->match_in_time_.trainIdx != -1){

            drawTrackingPointPattern(img_0_color_data, keypoint, this->GreenColor_);
        }
        else{

            drawTrackingPointPattern(img_0_color_data, keypoint, this->RedColor_);
        }
        
    }



    const std::shared_ptr<CameraFrame> &previous_camera_frame = camera_frame->ref_camera_frame_;
    if(previous_camera_frame){

        const std::shared_ptr<Image> &img_0_from_previous_camera_frame = previous_camera_frame->image_vector_.at(0);
        cv::Mat img_0_color_data_from_previous_camera_frame = img_0_from_previous_camera_frame->color_data_.clone();
    
        cv::Mat img_0_matches_in_time;
        cv::drawMatches(img_0_color_data, img_0->cv_keypoint_vector_, img_0_color_data_from_previous_camera_frame, img_0_from_previous_camera_frame->cv_keypoint_vector_, img_0->matches_in_time_, img_0_matches_in_time,
            cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
        );
    
    
        cv::imshow("img 0 matches in time", img_0_matches_in_time);

    }

  

}   


void OpenCVVisualizer::publishProjectedMapPoint(const std::shared_ptr<CameraFrame> &camera_frame){


    const std::shared_ptr<Image> img_0 = camera_frame->image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->color_data_.clone();

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img_0->sensor_id_)->getCVIntrinsicsMatrix();

    // const std::shared_ptr<Map> &map = camera_frame.getMap();
    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = this->map_->getMapPoints();

    for(const std::pair<const unsigned int, std::shared_ptr<MapPoint>> &item_pair: map_points){
       const std::shared_ptr<MapPoint> &map_point = item_pair.second;

       const Eigen::Vector3d pt3d_in_cam = img_0->T_c_w_ * map_point->pt3d_;
       cv::Point2d reprojected_pixel = camera2pixel(cv::Point3d(pt3d_in_cam.x(), pt3d_in_cam.y(), pt3d_in_cam.z()), cv_K);
       if(img_0->isInImage(reprojected_pixel) && pt3d_in_cam.z() > 0){
        cv::circle (img_0_color_data, reprojected_pixel, 2, cv::Scalar (0,255,0), -1);
       }

    }

    cv::imshow("projected map point on img " + std::to_string(img_0->sensor_id_), img_0_color_data);



}



void OpenCVVisualizer::publishSensorDepth(const std::shared_ptr<CameraFrame> &camera_frame){


    for(int i=0; i<(int)camera_frame->image_vector_.size(); i++){
        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);

        cv::Mat img_i_sensor_detph = img_i->sensor_depth_;
        cv::imshow("sensor depth in frame for image "+std::to_string(i), img_i_sensor_detph);
    }

}

void OpenCVVisualizer::publishLearnedDepth(const CameraFrame &camera_frame){


    for(int i=0; i<(int)camera_frame.image_vector_.size(); i++){
        const std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);

        if(img_i->learned_depth_.empty()){
            continue;
        }

        cv::Mat img_i_learned_detph = img_i->learned_depth_.clone();

        // visualize depth
        double min_val, max_val;
        cv::Mat depth_visual;
        cv::minMaxLoc(img_i_learned_detph, &min_val, &max_val);
        img_i_learned_detph = 255 * (img_i_learned_detph - min_val) / (max_val - min_val);
        img_i_learned_detph.convertTo(depth_visual, CV_8U);
        cv::applyColorMap(depth_visual, depth_visual, cv::COLORMAP_JET); //COLORMAP_HOT, COLORMAP_JET

        // Stack the image and depth map
        cv::Mat mixed_depth_color;

         // Blend images (alpha blending)
        double alpha = 0.80;  // depth overlay transparency (0 = invisible, 1 = fully depth)
        double beta = 1.0 - alpha;
        cv::addWeighted(depth_visual, alpha, img_i->color_data_, beta, 0.0, mixed_depth_color);

        cv::imshow("learned depth in frame for image " + std::to_string(i), mixed_depth_color);
    }

}



void OpenCVVisualizer::publishObjectDetection(const CameraFrame &camera_frame){

    for(int i=0; i<(int)camera_frame.image_vector_.size(); i++){

        const std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);
        if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_learned_object_detection_){


            cv::Mat img_i_color_data = img_i->color_data_.clone();

            // Show the results
            for (const BoxOutput& box_output : img_i->box_outputs_) {

                // Draw bounding box on image
                cv::rectangle(img_i_color_data, box_output.box_, cv::Scalar(0, 255, 0), 2);

                // Label
                std::string label = "ID: " + this->classes_[box_output.class_id_];
                cv::putText(img_i_color_data, label, cv::Point(box_output.box_.x, box_output.box_.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }

            cv::imshow("YOLO detections for image " + std::to_string(i), img_i_color_data);
        }

        

    }
    
}

void OpenCVVisualizer::publishSemanticSegmentation(const CameraFrame &camera_frame){

    for(int i=0; i<(int)camera_frame.image_vector_.size(); i++){

        const std::shared_ptr<Image> &img_i = camera_frame.image_vector_.at(i);

        if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_learned_semantic_segmentation_){

        

            cv::Mat img_i_color_data = img_i->color_data_;
            cv::Mat mask = img_i->color_data_.clone();
            for (const SegmentOutput& segment_output : img_i->segment_outputs_) {
                cv::rectangle(mask, segment_output.box_, cv::Scalar(0, 255, 0), 2, 8);
                mask(segment_output.box_).setTo(cv::Scalar(0, 0, 255), segment_output.boxMask_);

                std::string label = "ID: " + this->classes_[segment_output.class_id_];
                cv::putText(mask, label, cv::Point(segment_output.box_.x, segment_output.box_.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }

            cv::Mat mixed_segment_color;
            // Blend images (alpha blending)
            double alpha = 0.80;  // depth overlay transparency (0 = invisible, 1 = fully depth)
            double beta = 1.0 - alpha;
            cv::addWeighted(mask, alpha, img_i_color_data, beta, 0.0, mixed_segment_color);  

            cv::imshow("YOLO semantic segmentation for image " + std::to_string(i), mixed_segment_color);
        }

        
    }
}



void OpenCVVisualizer::publishTrackingInTime(const std::shared_ptr<CameraFrame> &camera_frame){

    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->color_data_.clone();


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



void OpenCVVisualizer::publish(const std::shared_ptr<CameraFrame> &camera_frame){



    if(this->sys_config_->visualizer_config_->opencv_params_->show_matching_in_frame_){
        publishMatchingInFrame(camera_frame);

    }

    if(this->sys_config_->visualizer_config_->opencv_params_->show_matching_in_time_){
        publishMatchingInTime(camera_frame);
    }
    if(this->sys_config_->visualizer_config_->opencv_params_->show_projected_mappoint_){
        publishProjectedMapPoint(camera_frame);
    }

    if(this->sys_config_->params_->check_triangulation_){
        
    }
   

    if(this->sys_config_->visualizer_config_->opencv_params_->show_sensor_depth_){

        publishSensorDepth(camera_frame);
    }




    if(this->sys_config_->visualizer_config_->opencv_params_->show_tracking_in_time_){

        publishTrackingInTime(camera_frame);
    }

    #ifdef USE_LIBTORCH
    if(this->sys_config_->visualizer_config_->opencv_params_->show_learned_depth_){
        publishLearnedDepth(camera_frame);
    }
   
    if(this->sys_config_->visualizer_config_->opencv_params_->show_object_detection_){

        publishObjectDetection(camera_frame);
    }    

    if(this->sys_config_->visualizer_config_->opencv_params_->show_semantic_segmentation_){

        publishSemanticSegmentation(camera_frame);
    }
    #endif


    cv::waitKey(1);



}
    
} // namespace modules_vins


