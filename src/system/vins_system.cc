#include "vins_system.h"


namespace modules_vins
{

System::System():
status_(Status::NOT_INITIALIZED)
{
    
    this->is_thread_running_.store(false, std::memory_order_relaxed);
    VLOG(KEY) << GREEN << "System has being initialized" << RESET;


}

void System::setConfig(const std::shared_ptr<SystemConfig> &config){
    this->config_ = config;
}

void System::setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh){
    this->nh_ = nh;
}

void System::setVisualizer(const std::shared_ptr<Visualizer> &visualizer){

    this->visualizer_ = visualizer;
}

void System::setInitializer(const std::shared_ptr<Initializer> &initializer){
    this->initializer_ = initializer;
}

void System::setVisualFrontend(const std::shared_ptr<VisualFrontend> &visual_frontend){


    this->visual_frontend_ = visual_frontend;
}


void System::setMap(const std::shared_ptr<Map> &map){
    this->state_.map_ = map;
}

void System::setRecorder(const std::shared_ptr<EVORecorder> &evo_recorder){

    this->evo_recorder_ = evo_recorder;
}

void System::RosMessagePtrToCvImageConstPtr(std::shared_ptr<rosbag::MessageInstance> &msg_ptr, cv_bridge::CvImageConstPtr &cv_ptr, const std::string &to_cv_dtype){


    const sensor_msgs::ImageConstPtr &image_msg_ptr = msg_ptr->instantiate<sensor_msgs::Image>();
    if (image_msg_ptr == nullptr) {
        VLOG(VERBOSE) << YELLOW << "message coming from camera is empty" << RESET;
        return;        
    }

    try {
        cv_ptr = cv_bridge::toCvShare(image_msg_ptr, to_cv_dtype);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}


void System::RosMessagePtrToCvImageConstPtr(std::shared_ptr<rosbag::MessageInstance> &msg_ptr, cv_bridge::CvImageConstPtr &cv_ptr){


    const sensor_msgs::ImageConstPtr &image_msg_ptr = msg_ptr->instantiate<sensor_msgs::Image>();
    if (image_msg_ptr == nullptr) {
        VLOG(VERBOSE) << YELLOW << "message coming from camera is empty" << RESET;
        return;        
    }

    try {
        cv_ptr = cv_bridge::toCvShare(image_msg_ptr, image_msg_ptr->encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}


void System::addCameraFrameDeque(const std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>> &msg_groups){


 
    std::vector<std::shared_ptr<Image>> image_vector;

    for(int cam_id=0; cam_id < (int)msg_groups.size(); cam_id++){

        // VLOG(VERBOSE) <<  "cam_id: " << cam_id <<  " : " <<msgs.at(cam_id).getTopic();

        
        std::map<std::string, std::shared_ptr<rosbag::MessageInstance>> dtype_to_msg_ptr_map = msg_groups.at(cam_id);

        cv_bridge::CvImageConstPtr cv_ptr;
        RosMessagePtrToCvImageConstPtr(dtype_to_msg_ptr_map["bgr"], cv_ptr, "bgr8");
        std::shared_ptr<Image> img = std::make_shared<Image>(cv_ptr->header.stamp.toSec(), cam_id, cv_ptr->image.clone());

        if(this->config_->camera_config_->params_vector_.at(cam_id)->use_sensor_depth_){
            RosMessagePtrToCvImageConstPtr(dtype_to_msg_ptr_map["depth"], cv_ptr);
            img->setSensorDepth(cv_ptr->image.clone());
        }

        image_vector.emplace_back(img);
        
    }

    
    const State &state = getState();

    std::shared_ptr<CameraFrame>camera_frame = std::make_shared<CameraFrame>(image_vector);
    camera_frame->setMap(state.map_);
    camera_frame->status_ = CameraFrame::Status::NORMAL;
    this->camera_frame_deque_.push_back(camera_frame);



}


void System::updateState(const std::shared_ptr<CameraFrame> &camera_frame){

    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }

    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    this->state_.timestamp_T_c_w_map_[img_0->timestamp_] = img_0->T_c_w_;
    VLOG(VERBOSE) << GREEN << "new state has been added to system" << RESET;

    // T_w_c is actual position and orientation of camera in world, for visualization
    Sophus::SE3<double> T_w_c_ = img_0->T_c_w_.inverse();
    this->evo_recorder_->writeTrajectoryOnce(img_0->timestamp_, T_w_c_.translation(), T_w_c_.unit_quaternion());


}


const State &System::getState() const{
    return this->state_;
}




void System::setGTState(const std::map<double, Sophus::SE3<double>> timestamp_GT_T_map){
    this->state_.timestamp_GT_T_full_map_ = timestamp_GT_T_map;

}


void System::callbackVisualNavigation(){

    bool is_thread_running = is_thread_running_.load(std::memory_order_relaxed);
    if(is_thread_running){
        return;
    }


    // std::thread thread([&] {
    //     this->is_thread_running_.store(true, std::memory_order_relaxed);

    //     // this->config_->params_->max_cameras_

    //     while(!this->camera_frame_deque_.empty()){

    //         CameraFrame camera_frame = this->camera_frame_deque_.at(0);
            

    //         this->visual_frontend_->pipeline(camera_frame);
    //         this->camera_frame_deque_.pop_front();


    //     }


    //     this->is_thread_running_.store(false, std::memory_order_relaxed);
    // });


    // thread.join();

    if(!this->camera_frame_deque_.empty()){
        

        std::shared_ptr<CameraFrame> &camera_frame = this->camera_frame_deque_.back();
        
        if(this->status_== Status::NOT_INITIALIZED){

            this->initializer_->pipeline(camera_frame);

            Initializer::Status initializer_status = this->initializer_->getStatus();
           
            // this->state_.map_->update(camera_frame);

        }


        // if(this->status_== SystemStatus::NORMAL){

        //     camera_frame->status_ = CameraFrame::Status::NORMAL;

        //     if(camera_frame->id_ == 0){
        //         this->initializer_->initializeGTTcwWithCameraFrame(camera_frame, this->state_);
        //     }

        //     this->visual_frontend_->pipeline(camera_frame);

        //     VisualFrontend::Status visual_frontend_status = this->visual_frontend_->getStatus();
            
        //     this->state_.map_->update(camera_frame);


        //     updateState(camera_frame);

        // }

        // this->visualizer_->publish(camera_frame, this->state_);


    }

}





} // namespace modules_vins