#include "opencv_visualizer.h"



namespace modules_vins
{

OpenCVVisualizer::OpenCVVisualizer(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config), GreenColor_(cv::Scalar(0,255,0)), RedColor_(cv::Scalar(0,0,255)), BlueColor_(cv::Scalar(255,0,0))
{

    // Note that in this example the classes are hard-coded
    this->classes_ = {"others","cover","grassland","chair"
        "bush",
        "tree trunk",
        "tree",
        "sky",
        "road",
        "grass",
        "flower bed",
        "swing",
        "dustbin",
        "water plants",
        "lamp",
        "column",
        "building",
        "water",
        "bridge",
        "facility",
        "fencing",
        "door",
        "sign",
        "person",
        "bike",
        "rider"};

    cv::RNG rng(12345);  // deterministic random
    
    for (int i = 0; i < this->classes_.size(); ++i) {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        this->class_colors_.emplace_back(b, g, r);  // OpenCV uses BGR
    }
    
      
}



void OpenCVVisualizer::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;
}

void OpenCVVisualizer::drawTrackingPointPattern(cv::Mat &img, const std::shared_ptr<KeyPoint> &keypoint, const cv::Scalar &color){

    const double r = 5;
    cv::Point2f pt2f_1,pt2f_2;
    
    pt2f_1.x=keypoint->cv_keypoint_.pt.x-r;
    pt2f_1.y=keypoint->cv_keypoint_.pt.y-r;
    pt2f_2.x=keypoint->cv_keypoint_.pt.x+r;
    pt2f_2.y=keypoint->cv_keypoint_.pt.y+r;

    cv::rectangle(img, pt2f_1, pt2f_2, color);
    cv::circle(img, keypoint->cv_keypoint_.pt, 2, color, -1);
    
    
}

void OpenCVVisualizer::publishMatchingInFrame(const std::shared_ptr<CameraFrame> &camera_frame){


    std::shared_ptr<Image> img_0;
    cv::Mat img_0_data;

    for(size_t i=0; i<(int)camera_frame->image_vector_.size(); i++){

        
        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);
        cv::Mat img_i_data = img_i->gray_data_.clone();

        if(i==0){
            img_0 = img_i;
            img_0_data = img_i_data;
        }
        
        
        // for(const std::shared_ptr<KeyPoint> &keypoint : img_i->keypoint_vector_){

        //     if(keypoint == nullptr)
        //     {
        //         continue;
        //     }

        //     if(keypoint->match_in_frame_.trainIdx != -1){

        //         drawTrackingPointPattern(img_i_data, keypoint, this->GreenColor_);
        //     }
        //     else{

        //         drawTrackingPointPattern(img_i_data, keypoint, this->RedColor_);
        //     }
        // }

 
        if(i>0){

            cv::Mat img_0_matches_in_frame;
            std::vector<cv::KeyPoint> cv_key_points_from_img0, cv_key_points_from_imgi;
            img_0->getCVKeyPoints(cv_key_points_from_img0);
            img_i->getCVKeyPoints(cv_key_points_from_imgi);

            // for(size_t idx = 0; idx < img_0->matches_in_frame_.size(); idx++){
            //     cv::DMatch match = img_0->matches_in_frame_.at(idx);
            //     cv_key_points_from_img0.push_back(img_0->keypoint_vector_.at(match.queryIdx)->cv_keypoint_);
            //     cv_key_points_from_imgi.push_back(img_i->keypoint_vector_.at(match.trainIdx)->cv_keypoint_);
            // }
    
            cv::drawMatches(img_i_data, cv_key_points_from_img0, img_i_data, cv_key_points_from_imgi, img_0->matches_in_frame_, img_0_matches_in_frame,
                cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
            );

            // cv::imshow("matches in frame between img 0 and img " + std::to_string(i) + " in frame id:" + std::to_string(camera_frame->id_), img_0_matches_in_frame);
            cv::imshow("matches in frame between img 0 and img " + std::to_string(i), img_0_matches_in_frame);

        }
        
    }



}

void OpenCVVisualizer::publishMatchingInTime(const std::shared_ptr<CameraFrame> &camera_frame){


    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->gray_data_.clone();


    // for(const std::shared_ptr<KeyPoint> &keypoint : img_0->keypoint_vector_){

    //     if(keypoint->match_in_time_.trainIdx != -1){

    //         drawTrackingPointPattern(img_0_color_data, keypoint, this->GreenColor_);
    //     }
    //     else{

    //         drawTrackingPointPattern(img_0_color_data, keypoint, this->RedColor_);
    //     }
        
    // }



    const std::shared_ptr<CameraFrame> &ref_camera_frame = camera_frame->ref_camera_frame_;
    if(ref_camera_frame){

        const std::shared_ptr<Image> &img_0_from_ref_camera_frame = ref_camera_frame->image_vector_.at(0);
        cv::Mat img_0_color_data_from_ref_camera_frame = img_0_from_ref_camera_frame->gray_data_.clone();
    
        cv::Mat img_0_matches_in_time;
         std::vector<cv::KeyPoint> cv_key_points_from_img0, cv_key_points_from_img0_ref_frame;
        img_0->getCVKeyPoints(cv_key_points_from_img0);
        img_0_from_ref_camera_frame->getCVKeyPoints(cv_key_points_from_img0_ref_frame);

        cv::drawMatches(img_0_color_data_from_ref_camera_frame, cv_key_points_from_img0_ref_frame, img_0_color_data, cv_key_points_from_img0, img_0_from_ref_camera_frame->matches_in_time_, img_0_matches_in_time,
            cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT
        );
    
    
        // cv::imshow("img 0 matches in time between ref frame id: " +  std::to_string(ref_camera_frame->id_) + " and current frame id: " + std::to_string(camera_frame->id_) , img_0_matches_in_time);
        cv::imshow("img 0 matches in time", img_0_matches_in_time);

    }

  

}   


void OpenCVVisualizer::publishProjectedMapPoint(const std::shared_ptr<CameraFrame> &camera_frame){


    const std::shared_ptr<Image> img_0 = camera_frame->image_vector_.at(0);
    cv::Mat img_0_color_data = img_0->color_data_.clone();

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img_0->sensor_id_)->getCVIntrinsicsMatrix();
    std::vector<double> valid_depths;
    std::vector<cv::Point2d> valid_pts2d;
    std::vector<Eigen::Vector3d> valid_pts3d_in_cam;

    double max_depth = -1;
    // const std::shared_ptr<Map> &map = camera_frame.getMap();
    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = this->map_->getMapPoints();

    for(const std::pair<const unsigned int, std::shared_ptr<MapPoint>> &item_pair: map_points){
       const std::shared_ptr<MapPoint> &map_point = item_pair.second;

       const Eigen::Vector3d pt3d_in_cam = img_0->T_c_w_ * map_point->pt3d_;
       const cv::Point2d reprojected_pixel = camera2pixel(cv::Point3d(pt3d_in_cam.x(), pt3d_in_cam.y(), pt3d_in_cam.z()), cv_K);
       const double d = pt3d_in_cam.z();
       if(img_0->isInImage(reprojected_pixel) && 
        d > this->sys_config_->params_->minimum_estimated_depth_ && 
        d < this->sys_config_->params_->maximum_estimated_depth_){

        valid_depths.emplace_back(d);
        valid_pts2d.emplace_back(reprojected_pixel);
        valid_pts3d_in_cam.emplace_back(pt3d_in_cam);

       }

    }

    if(valid_depths.size()>0){

        std::sort(valid_depths.begin(), valid_depths.end());
        size_t maximum_idx = static_cast<size_t>(0.99 * valid_depths.size());
        float depth_threshold = valid_depths[std::min(maximum_idx, valid_depths.size() - 1)];



        for(int i=0;i<valid_pts3d_in_cam.size();i++){

            const cv::Point2d valid_pt2d = valid_pts2d.at(i);
            const double d = valid_pts3d_in_cam.at(i).z();
            cv::Scalar color;
            if(d <= depth_threshold){

                float norm_d = d / depth_threshold;

                uchar blue  = static_cast<uchar>((1.0f - norm_d) * 255);
                uchar red   = static_cast<uchar>(norm_d * 255);
                uchar green = static_cast<uchar>((1.0f - std::abs(norm_d - 0.5f) * 2) * 255);
                color = cv::Scalar(red, green, blue);
            }
            else{
                // just want to show most of point, these white point also valid
                color = cv::Scalar(255,255,255);
            }
            
            cv::circle(img_0_color_data, valid_pt2d, 4, color, -1);

        }
    }
    cv::imshow("projected map point on img " + std::to_string(img_0->sensor_id_), img_0_color_data);



}


void OpenCVVisualizer::publishStereoDepth(const std::shared_ptr<CameraFrame> &camera_frame){

    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    if(img_0->stereo_depth_.empty()){
        return;
    }
    // visualize depth
    cv::Mat mixed_depth_color;
    invDepthAndMixColor(img_0->stereo_depth_, img_0->color_data_, mixed_depth_color);

    cv::imshow("disparity in frame for image "+std::to_string(0), mixed_depth_color);
    

}


void OpenCVVisualizer::publishSensorDepth(const std::shared_ptr<CameraFrame> &camera_frame){


    for(size_t i=0; i<(int)camera_frame->image_vector_.size(); i++){
        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);

        cv::Mat img_i_sensor_detph = img_i->sensor_depth_;
        if(img_i->sensor_depth_.empty()){
            continue;
        }


        cv::imshow("sensor depth in frame for image "+std::to_string(i), img_i_sensor_detph);
    }

}

void OpenCVVisualizer::publishLearnedDepth(const std::shared_ptr<CameraFrame> &camera_frame){


    for(size_t i=0; i<(int)camera_frame->image_vector_.size(); i++){
        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);

        if(img_i->learned_depth_.empty()){
            continue;
        }

        // cv::Mat img_i_learned_detph = img_i->learned_depth_.clone();

        // visualize depth
        cv::Mat mixed_depth_color;
        invDepthAndMixColor(img_i->learned_depth_, img_i->color_data_, mixed_depth_color);

        cv::imshow("learned depth in frame for image " + std::to_string(i), mixed_depth_color);
    }

}

void OpenCVVisualizer::publishLearnedStereoDisparity(const std::shared_ptr<CameraFrame> &camera_frame){


    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    if(img_0->learned_stereo_depth_.empty()){
        return;
    }
    // cv::Mat img_0_learned_stereo_depth = img_0->learned_stereo_depth_.clone();

    // visualize depth
    cv::Mat mixed_depth_color;
    invDepthAndMixColor(img_0->learned_stereo_depth_, img_0->color_data_, mixed_depth_color);

    cv::imshow("learned stereo disparity in frame for image " + std::to_string(0), mixed_depth_color);
    

}

void OpenCVVisualizer::invDepthAndMixColor(const cv::Mat &input_depth, const cv::Mat &input_color, cv::Mat &mixed_depth_color){

    // visualize depth
    double min_val, max_val;
    cv::Mat inv_depth = cv::Mat::zeros(input_depth.size(), CV_32FC1);

    for (int y = 0; y < input_depth.rows; ++y) {
        for (int x = 0; x < input_depth.cols; ++x) {
            float d = input_depth.at<float>(y, x);
            if (d > 1e-6f) {  // avoid zero or negative depth
                inv_depth.at<float>(y, x) = 1.0f / d;
            } else {
                inv_depth.at<float>(y, x) = std::numeric_limits<float>::quiet_NaN();  
            }
        }
    }

    cv::Mat depth_visual;
    cv::minMaxLoc(inv_depth, &min_val, &max_val);
    inv_depth = 255 * (inv_depth - min_val) / (max_val - min_val);
    inv_depth.convertTo(depth_visual, CV_8U);
    cv::applyColorMap(depth_visual, depth_visual, cv::COLORMAP_JET); //COLORMAP_HOT, COLORMAP_JET

     // Blend images (alpha blending)
    double alpha = 1.0;  // depth overlay transparency (0 = invisible, 1 = fully depth)
    double beta = 1.0 - alpha;
    cv::addWeighted(depth_visual, alpha, input_color, beta, 0.0, mixed_depth_color);

}



void OpenCVVisualizer::publishObjectDetection(const std::shared_ptr<CameraFrame> &camera_frame){

    for(size_t i=0; i<(int)camera_frame->image_vector_.size(); i++){

        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);
        if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_learned_object_detection_){


            cv::Mat img_i_color_data = img_i->color_data_.clone();

            // Show the results
            for (const BoxOutput& box_output : img_i->box_outputs_) {

                std::string detected_class = this->classes_[box_output.class_id_];
                // if(detected_class == "tree"){
                //      // Draw bounding box on image
                //     cv::rectangle(img_i_color_data, box_output.box_, this->class_colors_[box_output.class_id_], 2);

                //     // Label
                //     std::string label = "ID: " + detected_class;
                //     cv::putText(img_i_color_data, label, cv::Point(box_output.box_.x, box_output.box_.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, this->class_colors_[box_output.class_id_], 1);
                // }
                cv::rectangle(img_i_color_data, box_output.box_, this->class_colors_[box_output.class_id_], 2);

                // Label
                std::string label = "ID: " + detected_class;
                cv::putText(img_i_color_data, label, cv::Point(box_output.box_.x, box_output.box_.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, this->class_colors_[box_output.class_id_], 1);
            }

            cv::imshow("YOLO detections for image " + std::to_string(i), img_i_color_data);
        }

        

    }
    
}

void OpenCVVisualizer::publishSemanticSegmentation(const std::shared_ptr<CameraFrame> &camera_frame){

    for(size_t i=0; i<(int)camera_frame->image_vector_.size(); i++){

        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);

        if(this->sys_config_->camera_config_->params_vector_.at(img_i->sensor_id_)->use_learned_semantic_segmentation_){

        

            cv::Mat img_i_color_data = img_i->color_data_;
            cv::Mat mask = img_i->color_data_.clone();
            for (const SegmentOutput& segment_output : img_i->segment_outputs_) {

                std::string segment_out_class = this->classes_[segment_output.class_id_];
                // if(segment_out_class == "tree"){

                //     cv::rectangle(mask, segment_output.box_, this->class_colors_[segment_output.class_id_], 2, 8);
                //     mask(segment_output.box_).setTo(this->class_colors_[segment_output.class_id_], segment_output.boxMask_);

                //     std::string label = "ID: " + this->classes_[segment_output.class_id_];
                //     cv::putText(mask, label, cv::Point(segment_output.box_.x, segment_output.box_.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, this->class_colors_[segment_output.class_id_], 1);
                // }
               

                cv::rectangle(mask, segment_output.box_, this->class_colors_[segment_output.class_id_], 2, 8);
                mask(segment_output.box_).setTo(this->class_colors_[segment_output.class_id_], segment_output.boxMask_);

                std::string label = "ID: " + this->classes_[segment_output.class_id_];
                cv::putText(mask, label, cv::Point(segment_output.box_.x, segment_output.box_.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, this->class_colors_[segment_output.class_id_], 1);
                
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


    if(camera_frame->ref_camera_frame_){

        const std::shared_ptr<Image> &img_0 = camera_frame->ref_camera_frame_->image_vector_.at(0);
        cv::Mat img_0_color_data = img_0->color_data_.clone();


        for(const std::shared_ptr<KeyPoint> &keypoint : img_0->keypoint_vector_){

            if(keypoint->match_in_time_.trainIdx != -1){

                drawTrackingPointPattern(img_0_color_data, keypoint, this->GreenColor_);
            }
            else{

                drawTrackingPointPattern(img_0_color_data, keypoint, this->RedColor_);
            }
            
        }

        cv::imshow("tracking point in time for img 0 from reference frame", img_0_color_data);

    }
    

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

    if(this->sys_config_->visualizer_config_->opencv_params_->show_stereo_depth_){
        publishStereoDepth(camera_frame);
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

    if(this->sys_config_->visualizer_config_->opencv_params_->show_learned_stereo_disparity_){

        publishLearnedStereoDisparity(camera_frame);
    }

    
    #endif


    cv::waitKey(this->sys_config_->visualizer_config_->opencv_params_->cv_waitkey_num_);



}
    
} // namespace modules_vins


