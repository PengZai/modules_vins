#include "vins_system.h"


namespace modules_vins
{

System::System():
status_(Status::NOT_INITIALIZED)
{
    
    this->is_thread_running_.store(false, std::memory_order_relaxed);
    LOG(INFO) << GREEN << "System has being initialized" << RESET;


}

void System::setConfig(const std::shared_ptr<SystemConfig> &config){
    this->config_ = config;
}

void System::setState(const std::shared_ptr<State> &state){
    this->state_ = state;
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

void System::setKeyFrameManager(const std::shared_ptr<KeyFrameManager> &key_frame_manager){

    this->key_frame_manager_ = key_frame_manager;
}


void System::setMap(const std::shared_ptr<Map> &map){
    this->state_->map_ = map;
}

void System::setRecorder(const std::shared_ptr<EVORecorder> &evo_recorder){

    this->evo_recorder_ = evo_recorder;
}

void System::RosMessagePtrToCvImageConstPtr(std::shared_ptr<rosbag::MessageInstance> &msg_ptr, cv_bridge::CvImageConstPtr &cv_ptr, const std::string &to_cv_dtype){


    const sensor_msgs::ImageConstPtr &image_msg_ptr = msg_ptr->instantiate<sensor_msgs::Image>();
    if (image_msg_ptr == nullptr) {
        LOG(INFO) << YELLOW << "message coming from camera is empty" << RESET;
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
        LOG(INFO) << YELLOW << "message coming from camera is empty" << RESET;
        return;        
    }

    try {
        cv_ptr = cv_bridge::toCvShare(image_msg_ptr, image_msg_ptr->encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}


void System::addFrameDeque(const std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>> &msg_groups){


 
    std::vector<std::shared_ptr<Image>> image_vector;
    std::shared_ptr<Frame> frame = std::make_shared<Frame>();


    for(int cam_id=0; cam_id < (int)msg_groups.size(); cam_id++){

        // LOG(INFO) <<  "cam_id: " << cam_id <<  " : " <<msgs.at(cam_id).getTopic();

        
        std::map<std::string, std::shared_ptr<rosbag::MessageInstance>> dtype_to_msg_ptr_map = msg_groups.at(cam_id);

        cv_bridge::CvImageConstPtr cv_ptr;
        RosMessagePtrToCvImageConstPtr(dtype_to_msg_ptr_map["bgr"], cv_ptr, "bgr8");
        std::shared_ptr<Image> img = std::make_shared<Image>(cv_ptr->header.stamp.toSec(), cam_id, cv_ptr->image.clone());

        if(this->config_->camera_config_->getParamsAt<CameraParameters>(cam_id)->use_sensor_depth_){
            RosMessagePtrToCvImageConstPtr(dtype_to_msg_ptr_map["depth"], cv_ptr);
            img->setSensorDepth(cv_ptr->image.clone());
        }

        img->setFrame(frame);
        image_vector.emplace_back(img);
        
    }

    
    const std::shared_ptr<State> &state = getState();

    frame->setImages(image_vector);
    frame->status_ = Frame::Status::NORMAL;
    frame->timestamp_ = image_vector.at(0)->timestamp_;
    frame->use_comparison_pose_for_pose_estimation_ = this->config_->params_->use_comparison_pose_for_pose_estimation_;
    frame->comparison_pose_idx_for_pose_estimation_ = this->config_->params_->comparison_pose_idx_for_pose_estimation_;

    this->frame_deque_.push_back(frame);



}


void System::updateState(const std::shared_ptr<Frame> &frame){

    if(frame->status_ != Frame::Status::NORMAL){
        return;
    }


    this->state_->timestamp_T_b_w_map_[frame->timestamp_] = frame->T_b_w_;
    LOG(INFO) << GREEN << "new state has been added to system" << RESET;

    if(frame->is_key_frame_){
        this->state_->timestamp_key_T_b_w_map_[frame->timestamp_] = frame->T_b_w_;
        LOG(INFO) << GREEN << "new key camera frame has been added to system" << RESET;
    }

    // T_w_c is actual position and orientation of camera in world, because Pw = T_w_c * Pc, 
    // that means T_w_c is far away from origin
    Sophus::SE3<double> T_w_b_ = frame->T_b_w_.inverse();
    this->evo_recorder_->writeTrajectoryOnce(frame->timestamp_, T_w_b_.translation(), T_w_b_.unit_quaternion());


}


const std::shared_ptr<State> &System::getState() const{
    return this->state_;
}






void System::callbackVisualNavigation(){

    bool is_thread_running = is_thread_running_.load(std::memory_order_relaxed);
    if(is_thread_running){
        return;
    }


    // std::thread thread([&] {
    //     this->is_thread_running_.store(true, std::memory_order_relaxed);

    //     // this->config_->params_->max_cameras_

    //     while(!this->frame_deque_.empty()){

    //         Frame frame = this->frame_deque_.at(0);
            

    //         this->visual_frontend_->pipeline(frame);
    //         this->frame_deque_.pop_front();


    //     }


    //     this->is_thread_running_.store(false, std::memory_order_relaxed);
    // });


    // thread.join();

    if(!this->frame_deque_.empty()){
        

        std::shared_ptr<Frame> &frame = this->frame_deque_.back();



        frame->status_ = Frame::Status::NORMAL;

        this->visual_frontend_->pipeline(frame);

        VisualFrontend::Status visual_frontend_status = this->visual_frontend_->getStatus();
        if(visual_frontend_status  == VisualFrontend::Status::NORMAL && this->status_ == Status::NOT_INITIALIZED){

            const std::deque<std::shared_ptr<Frame>> &visual_frontend_ref_frame_deque = this->visual_frontend_->getRefFrameDeque();
            this->status_ = Status::NORMAL;
        }   

        
        if(this->status_ == Status::NORMAL && frame->status_ == Frame::Status::NORMAL){
            key_frame_manager_->updateKeyFrame(frame); 
            if(frame->is_key_frame_){
                LOG(INFO) <<  "camera_id: " << frame->id_ << " is a key camera frame";
                key_frame_manager_->reconstructInKeyFrame(frame);
            }                   
            this->state_->map_->update(frame);
            updateState(frame);
        }

        if(visual_frontend_status == VisualFrontend::Status::GET_LOST){

            this->status_= Status::NOT_INITIALIZED;
            LOG(INFO) << RED << "get lost, fail" << RESET;
            std::exit(0);

        }


        

   

        this->visualizer_->publish(frame, this->state_);
        LOG(INFO) << GREEN << "finished process camera frame " << frame->id_ << RESET;

    }

}





} // namespace modules_vins