#include "vins_system.h"


namespace modules_vins
{

System::System():
is_initialized_(false)
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
    this->map_ = map;
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

    
    CameraFrame camera_frame(image_vector);
    camera_frame.setMap(this->map_);
    this->camera_frame_deque_.push_back(camera_frame);


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

    while(!this->camera_frame_deque_.empty()){

        CameraFrame &camera_frame = this->camera_frame_deque_.front();
        
        if(this->is_initialized_ == false){
            this->is_initialized_ = this->initializer_->initialize(camera_frame);
        }

        if(this->is_initialized_ == true){

            this->visual_frontend_->pipeline(camera_frame);

        }

        this->visualizer_->publish(camera_frame);

        this->camera_frame_deque_.pop_front();



    }

}





} // namespace modules_vins