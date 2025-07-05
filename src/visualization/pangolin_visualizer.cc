#include "pangolin_visualizer.h"




namespace modules_vins{



PangolinVisualizer::PangolinVisualizer(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{


    this->frame_size_ = sys_config->visualizer_config_->pangolin_params_->frame_size_;
    this->point_size_ = sys_config->visualizer_config_->pangolin_params_->point_size_;
    this->trajectory_line_size_ = sys_config->visualizer_config_->pangolin_params_->trajectory_line_size_;
    this->viewer_eye_positionX_ = sys_config->visualizer_config_->pangolin_params_->viewer_eye_positionX_;
    this->viewer_eye_positionY_ = sys_config->visualizer_config_->pangolin_params_->viewer_eye_positionY_;
    this->viewer_eye_positionZ_ = sys_config->visualizer_config_->pangolin_params_->viewer_eye_positionZ_;


    const double window_width = 1080.0;
    const double window_height = 720.0;


    pangolin::CreateWindowAndBind("Pangolin Viewer", window_width, window_height);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    // Define projection and initial model view matrix
    this->s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(window_width,window_height,500,500,512,389,0.1,1000),
        pangolin::ModelViewLookAt(
            this->viewer_eye_positionX_,this->viewer_eye_positionY_,this->viewer_eye_positionZ_,  // eye (camera) position in world
            0, 0, 0,  // look-at point (center of scene)
            0.0, -1.0, 0.0 // up direction (usually Y-up)
        )
    );


    this->d_cam_ = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -window_width/window_height)
        .SetHandler(new pangolin::Handler3D(this->s_cam_));


    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

    this->is_reset_ = new pangolin::Var<bool>("menu.Reset", false);
    this->is_follow_camera_ = new pangolin::Var<bool>("menu.Follow Camera",true,true);



}


void PangolinVisualizer::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;
}


void PangolinVisualizer::publish(const std::shared_ptr<State> &state){


    if(!pangolin::ShouldQuit()){

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            this->d_cam_.Activate(this->s_cam_);

            glClearColor(1.0f,1.0f,1.0f,1.0f);

            // Draw axis, red - x green - y blue -z
            pangolin::glDrawAxis(0.5);

            publishPoses(state);
            publishKeyPoses(state);
            publishMapPoints(state);
            publishTrajectories(state);
        

        pangolin::FinishFrame();
    }
}


void PangolinVisualizer::publishKeyPoses(const std::shared_ptr<State> &state){

    if(!state->timestamp_key_T_b_w_map_.empty()){

        for (const auto& [timestamp, T_b_w] : state->timestamp_key_T_b_w_map_) {

            drawFrame(T_b_w.inverse().matrix(), Eigen::Vector3i(0,255,0));

        }


    }
}



void PangolinVisualizer::publishPoses(const std::shared_ptr<State> &state){


    if(!state->timestamp_T_b_w_map_.empty()){

        auto it = state->timestamp_T_b_w_map_.rbegin();
        const double newest_timestamp = it->first;
        const Sophus::SE3<double> &newest_T_b_w = it->second;

        drawFrame(newest_T_b_w.inverse().matrix(), Eigen::Vector3i(0,255,0));

        if(*is_follow_camera_){
            s_cam_.Follow(newest_T_b_w.inverse().matrix());
        }


         if(this->sys_config_->visualizer_config_->rviz_params_->show_comparison_pose_){

            for(size_t idx=0; idx < this->sys_config_->comparison_config_->params_vector_.size(); idx++){

                std::map<double, Sophus::SE3<double>>  timestamp_pose_comparison_in_base_map = state->timestamp_pose_comparison_in_base_map_vector_.at(idx);

                double synchronized_comparison_pose_timestamp = state->getSynchronizedPoseTimestamp(newest_timestamp, 
                    this->sys_config_->comparison_config_->getParamsAt<ComparisonParameters>(idx)->max_tolerant_time_offset_, 
                    state->timestamp_pose_comparison_in_comparison_full_map_vector_.at(idx));

                if(synchronized_comparison_pose_timestamp == -1){
                    return;
                }
                const Sophus::SE3<double> &synchronized_pose_comparison_in_base = timestamp_pose_comparison_in_base_map.at(synchronized_comparison_pose_timestamp);

                Eigen::VectorXd color = this->sys_config_->comparison_config_->getParamsAt<ComparisonParameters>(idx)->color_;
                drawFrame(synchronized_pose_comparison_in_base.inverse().matrix(), Eigen::Vector3i(color(0), color(1), color(2)));
            }

        }
    }
}

void PangolinVisualizer::publishTrajectories(const std::shared_ptr<State> &state){

    if(!state->timestamp_T_b_w_map_.empty()){
         publishTrajectory(state->timestamp_T_b_w_map_, Eigen::Vector3i(0,255,0));
         if(this->sys_config_->visualizer_config_->pangolin_params_->show_comparison_trajectory_){

            auto it = state->timestamp_T_b_w_map_.rbegin();
            const double newest_timestamp = it->first;

            for(size_t idx=0; idx < this->sys_config_->comparison_config_->params_vector_.size(); idx++){

                std::map<double, Sophus::SE3<double>>  timestamp_pose_comparison_in_base_map = state->timestamp_pose_comparison_in_base_map_vector_.at(idx);

                double synchronized_timestamp_for_pose_comparison = state->getSynchronizedPoseTimestamp(newest_timestamp, 
                    this->sys_config_->comparison_config_->getParamsAt<ComparisonParameters>(idx)->max_tolerant_time_offset_, 
                    state->timestamp_pose_comparison_in_comparison_full_map_vector_.at(idx));
                    
                if(synchronized_timestamp_for_pose_comparison == -1){
                    return;
                }

                auto it_end = timestamp_pose_comparison_in_base_map.find(synchronized_timestamp_for_pose_comparison);

                std::map<double, Sophus::SE3<double>> timestamp_pose_comparison_in_base_sub_map(timestamp_pose_comparison_in_base_map.begin(), it_end);
                
                Eigen::VectorXd color = this->sys_config_->comparison_config_->getParamsAt<ComparisonParameters>(idx)->color_;
                publishGTTrajectory(timestamp_pose_comparison_in_base_sub_map, Eigen::Vector3i(color(0), color(1), color(2)));
            }  
         } 
    }

}



void PangolinVisualizer::publishTrajectory(std::map<double, Sophus::SE3<double>> timestamp_T_c_w_map, const Eigen::Vector3i &bgr){

    bool is_first = true;
    Eigen::Vector3d prev_translation; 

    if(!timestamp_T_c_w_map.empty()){

        for (const auto& [timestamp, T_c_w] : timestamp_T_c_w_map) {
            
            const Eigen::Vector3d &translation = T_c_w.inverse().translation();
            drawPoint(translation, bgr);

            if(is_first == true){
                is_first = false;
                prev_translation = translation;        
                continue;
            }

            drawLine(prev_translation, translation, bgr);
            
            prev_translation = translation;        

        }

    }

}


void PangolinVisualizer::publishGTTrajectory(std::map<double, Sophus::SE3<double>> timestamp_T_c_w_map, const Eigen::Vector3i &bgr){

    bool is_first = true;
    Eigen::Vector3d prev_translation; 

    if(!timestamp_T_c_w_map.empty()){

        for (const auto& [timestamp, T_c_w] : timestamp_T_c_w_map) {
            
            const Eigen::Vector3d &translation = T_c_w.inverse().translation();
            drawPoint(translation, bgr);

            if(is_first == true){
                is_first = false;
                prev_translation = translation;        
                continue;
            }

            drawLine(prev_translation, translation, bgr);
            
            prev_translation = translation;        

        }

    }

}


void PangolinVisualizer::drawLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3i &bgr){

    glLineWidth(this->trajectory_line_size_);
    glColor3d(bgr[2]/255.0f,bgr[1]/255.0f,bgr[0]/255.0f);
    glBegin(GL_LINES);
    glVertex3d(p1.x(),p1.y(),p1.z());
    glVertex3d(p2.x(),p2.y(),p2.z());
    glEnd();
}

void PangolinVisualizer::drawFrame(const Eigen::Matrix4d &T_w_c, const Eigen::Vector3i &bgr, bool drawAxis){


    const float w = this->frame_size_;
    const float h = w;
    const float z = 2*w;
    const float frame_line_width = 2.0;


    glPushMatrix();

    glMultMatrixd((GLdouble*)T_w_c.data());

    glPointSize(10.0f);  // Set point size in pixels
    glBegin(GL_POINTS);
    glColor3d(bgr[2]/255.0f,bgr[1]/255.0f,bgr[0]/255.0f);   // Set point color (red)
    glVertex3d(0, 0, 0);  // Plot point at (x=0, y=0, z=0)
    glEnd();

    // Draw axis, red - x green - y blue -z
    if(drawAxis){
        pangolin::glDrawAxis(this->frame_size_);
    }

    glLineWidth(frame_line_width);
    glColor3d(bgr[2]/255.0f,bgr[1]/255.0f,bgr[0]/255.0f);
    glBegin(GL_LINES);

    glVertex3d(0,0,0);
    glVertex3d(w,h,z);
    glVertex3d(0,0,0);
    glVertex3d(w,-h,z);
    glVertex3d(0,0,0);
    glVertex3d(-w,-h,z);
    glVertex3d(0,0,0);
    glVertex3d(-w,h,z);
    
    glVertex3d(w,h,z);
    glVertex3d(w,-h,z);
    glVertex3d(-w,h,z);
    glVertex3d(-w,-h,z);
    glVertex3d(-w,h,z);
    glVertex3d(w,h,z);
    glVertex3d(-w,-h,z);
    glVertex3d(w,-h,z);
    glEnd();

    glColor4f(bgr[2]/255.0f,bgr[1]/255.0f,bgr[0]/255.0f, 0.3f); // RGBA
    glBegin(GL_QUADS);
    glVertex3d(-w, -h, z); // bottom-left
    glVertex3d(w, -h, z);  // bottom-right
    glVertex3d(w, h, z);   // top-right
    glVertex3d(-w, h, z);  // top-left
    glEnd();

    glPopMatrix();

    glEnd();


}


void PangolinVisualizer::publishMapPoints(const std::shared_ptr<State> &state){


    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = state->map_->getMapPoints();

    for(const std::pair<const unsigned int, std::shared_ptr<MapPoint>> &item_pair: map_points){
       const std::shared_ptr<MapPoint> &map_point = item_pair.second;


       drawPoint(map_point->pt3d_, map_point->bgr_);


    }

}

void PangolinVisualizer::drawPoint(const Eigen::Vector3d &pt3d, const Eigen::Vector3i &bgr){

    glPointSize(this->point_size_);
    glBegin(GL_POINTS);
    glVertex3d(pt3d[0],pt3d[1],pt3d[2]);
    glColor3d(bgr[2]/255.0f,bgr[1]/255.0f,bgr[0]/255.0f);
    glEnd();

}


} // namespace modules_vins