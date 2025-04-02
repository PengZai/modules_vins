#include "pangolin_visualizer.h"




namespace modules_vins{



PangolinVisualizer::PangolinVisualizer(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{


    this->frame_size_ = sys_config->visualizer_config_->pangolin_params_->frame_size_;
    this->point_size_ = sys_config->visualizer_config_->pangolin_params_->point_size_;
    this->viewer_eye_positionX_ = sys_config->visualizer_config_->pangolin_params_->viewer_eye_positionX_;
    this->viewer_eye_positionY_ = sys_config->visualizer_config_->pangolin_params_->viewer_eye_positionY_;
    this->viewer_eye_positionZ_ = sys_config->visualizer_config_->pangolin_params_->viewer_eye_positionZ_;


    const double window_width = 1080.0;
    const double window_height = 720.0;





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


void PangolinVisualizer::publish(const CameraFrame &camera_frame){

    

    if(!pangolin::ShouldQuit()){
    
        const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        this->d_cam_.Activate(this->s_cam_);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        // Draw something
        pangolin::glDrawAxis(2.0);

        drawFrame(img_0->T_c_w_.matrix());

        if(*is_follow_camera_){
            s_cam_.Follow(img_0->T_c_w_.matrix());
        }

        drawMapPoints(camera_frame);

        std::cout << "is_follow_camera_: " << pangolin::Var<bool>("menu.Follow Camera") << std::endl;

        pangolin::FinishFrame();
    }
}

void PangolinVisualizer::drawFrame(const Eigen::Matrix4d &T_w_c){

    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    const float w = this->frame_size_;
    const float h = w;
    const float z = 2*w;
    const float frame_line_width = 2.0;


    tmp.setIdentity();


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


}


void PangolinVisualizer::drawMapPoints(const CameraFrame &camera_frame){


    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = camera_frame.getMap()->getMapPoints();

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