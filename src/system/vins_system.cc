#include "vins_system.h"


namespace modules_vins
{

System::System(){
    
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

void System::setVisualFrontend(const std::shared_ptr<VisualFrontend> &visual_frontend){


    this->visual_frontend_ = visual_frontend;
}



void System::addCameraFrameDeque(const std::vector<rosbag::MessageInstance> &msgs){


 
    std::vector<Image> image_vector;

    for(int cam_id=0; cam_id < (int)msgs.size(); cam_id++){

        VLOG(VERBOSE) <<  "cam_id: " << cam_id <<  " : " <<msgs.at(cam_id).getTopic();
        
        sensor_msgs::Image::ConstPtr img_msg = msgs.at(cam_id).instantiate<sensor_msgs::Image>();
        if (img_msg == nullptr) {
            VLOG(VERBOSE) << YELLOW << "message coming from camera " << cam_id << " is empty" << RESET;
            return;        
        }

        // cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }


        Image img(cv_ptr->header.stamp.toSec(), cam_id, cv_ptr->image.clone());
        image_vector.emplace_back(img);
        
    }

    
    CameraFrame camera_frame(image_vector);
    this->camera_frame_deque_.push_back(camera_frame);


}


// void System::addCameraFrameDeque(const std::vector<rosbag::MessageInstance> &msgs){


 
//     std::vector<Image> image_vector;
//     cv_bridge::CvImageConstPtr cv_ptr0;
//     cv_bridge::CvImageConstPtr cv_ptr1;


//     for(int cam_id=0; cam_id < (int)msgs.size(); cam_id++){

//         sensor_msgs::Image::ConstPtr img_msg = msgs.at(cam_id).instantiate<sensor_msgs::Image>();
//         if (img_msg == nullptr) {
//             VLOG(VERBOSE) << YELLOW << "message coming from camera " << cam_id << " is empty" << RESET;
//             return;        
//         }
//         if(cam_id==0){
//             cv_ptr0 = cv_bridge::toCvShare(img_msg, "bgr8");
//             try {
//                 cv_ptr0 = cv_bridge::toCvShare(img_msg, "bgr8");
//             } catch (cv_bridge::Exception& e) {
//                 ROS_ERROR("cv_bridge exception: %s", e.what());
//             }
//             Image img(cv_ptr0->header.stamp.toSec(), cam_id, cv_ptr0->image.clone());
//             image_vector.emplace_back(img);

//         }
//         else{
//             cv_ptr1 = cv_bridge::toCvShare(img_msg, "bgr8");
//             try {
//                 cv_ptr1 = cv_bridge::toCvShare(img_msg, "bgr8");
//             } catch (cv_bridge::Exception& e) {
//                 ROS_ERROR("cv_bridge exception: %s", e.what());
//             }
//             Image img(cv_ptr1->header.stamp.toSec(), cam_id, cv_ptr1->image.clone());
//             image_vector.emplace_back(img);

//         }
//         // cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        

        
        
//     }


//     CameraFrame camera_frame(image_vector);
//     this->camera_frame_deque_.push_back(camera_frame);


// }

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
        

        this->visual_frontend_->pipeline(camera_frame);

        this->visualizer_->publish(camera_frame);


        this->camera_frame_deque_.pop_front();



    }

}





} // namespace modules_vins