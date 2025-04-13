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


    // // visualization
    // vis = new cv::viz::Viz3d("Visual Odometry");
    // cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    // cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    // cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    // vis->setViewerPose( cam_pose );
    
    // world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    // camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    // vis->showWidget( "World", world_coor );
    // vis->showWidget( "Camera", camera_coor );



    pangolin::CreateWindowAndBind("Pangolin Viewer", window_width, window_height);

    glEnable(GL_DEPTH_TEST);



    // Define projection and initial model view matrix
    this->s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(window_width,window_height,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(
            this->viewer_eye_positionX_,this->viewer_eye_positionY_,this->viewer_eye_positionZ_,  // eye (camera) position in world
            0, 0, 0,  // look-at point (center of scene)
            0.0, 1.0, 0.0 // up direction (usually Y-up)
        )
    );


    this->d_cam_ = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -window_width/window_height)
        .SetHandler(new pangolin::Handler3D(this->s_cam_));


    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

    this->is_reset_ = new pangolin::Var<bool>("menu.Reset", false);
    this->is_follow_camera_ = new pangolin::Var<bool>("menu.Follow Camera",false,true);



}


void PangolinVisualizer::publish(const State &state){


    if(!pangolin::ShouldQuit()){
    
        auto it = state.timestamp_T_c_w_map_.rbegin();
        const double newest_timestamp = it->first;
        const Sophus::SE3<double> &newest_T_c_w = it->second;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        this->d_cam_.Activate(this->s_cam_);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        // Draw something
        pangolin::glDrawAxis(2.0);

        drawFrame(newest_T_c_w.inverse().matrix());

        if(*is_follow_camera_){
            s_cam_.Follow(newest_T_c_w.matrix());
        }

        drawMapPoints(state);
        drawTrajectory(state);

        pangolin::FinishFrame();
    }
}

void PangolinVisualizer::drawTrajectory(const State &state){

    // ref_translation_ = state.T_c_w_vector_.at(0).translation();
    // for(int i=1; i < (int)state.T_c_w_vector_.size();i++){

    //     const Sophus::SE3<double> &T_c_W = state.T_c_w_vector_.at(i).inverse();

    //     glLineWidth(this->trajectory_line_size_);
    //     glColor4f(0.0f,1.0f,0.0f,0.6f);
    //     glBegin(GL_LINES);
    //     Eigen::Vector3d translation = T_c_W.translation();
    //     glVertex3d(ref_translation_.x(),ref_translation_.y(),ref_translation_.z());
    //     glVertex3d(translation.x(),translation.y(),translation.z());
    //     glEnd();
    //     ref_translation_ = translation;        
    // } 

}

void PangolinVisualizer::drawFrame(const Eigen::Matrix4d &T_w_c){


    const float w = this->frame_size_;
    const float h = w;
    const float z = 2*w;
    const float frame_line_width = 2.0;


    glPushMatrix();

    glMultMatrixd((GLdouble*)T_w_c.data());

    glPointSize(10.0f);  // Set point size in pixels
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f);   // Set point color (red)
    glVertex3d(0, 0, 0);  // Plot point at (x=0, y=0, z=0)
    glEnd();

    glLineWidth(frame_line_width);
    glColor3f(0.0f,0.0f,1.0f);
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

    glPopMatrix();

    glEnd();


    // cv::Affine3d M(
    //     cv::Affine3d::Mat3( 
    //         T_w_c(0,0), T_w_c(0,1), T_w_c(0,2),
    //         T_w_c(1,0), T_w_c(1,1), T_w_c(1,2),
    //         T_w_c(2,0), T_w_c(2,1), T_w_c(2,2)
    //     ), 
    //     cv::Affine3d::Vec3(
    //         T_w_c(0,3), T_w_c(1,3), T_w_c(2,3)
    //     )
    // );

    // vis->setWidgetPose( "Camera", M);
    // vis->spinOnce(1, false);


}


void PangolinVisualizer::drawMapPoints(const State &state){


    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = state.map_->getMapPoints();

    for(const std::pair<const unsigned int, std::shared_ptr<MapPoint>> &item_pair: map_points){
       const std::shared_ptr<MapPoint> &map_point = item_pair.second;

 
       glPointSize(this->point_size_);
       glBegin(GL_POINTS);
       glColor3d(map_point->bgr_[0]/255.0d,map_point->bgr_[1]/255.0d,map_point->bgr_[2]/255.0d);
       glVertex3d(map_point->pt3d_[0], map_point->pt3d_[1], map_point->pt3d_[2]);
       glEnd();


    }

}


} // namespace modules_vins