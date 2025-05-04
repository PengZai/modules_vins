#include "dataloader.h"


namespace modules_vins
{


FileDataLoader::FileDataLoader(const std::shared_ptr<SystemConfig> &sys_config):
sys_config_(sys_config)
{
    
}


void FileDataLoader::load_groundtruth(const std::string &path_to_file, std::map<double, Sophus::SE3<double>> &timestamp_GT_T_map){



    std::ifstream file_in(path_to_file);
    if(!file_in){
        VLOG(VERBOSE) << RED << "File not found: " << path_to_file <<  RESET;
    }

    std::string line;


    while(std::getline(file_in, line)){
        //Skip comments
        if(line.empty() || line[0] == '#'){
            continue;
        } 
        std::istringstream iss(line); 
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Vector3d t(tx, ty, tz);

        Sophus::SE3<double> T_c_w(q,t);
        timestamp_GT_T_map[timestamp] = T_c_w;
    }



}

ROSDataLoader::ROSDataLoader(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh):
sys_config_(sys_config), 
nh_(nh), 
path_to_bag_("unset_path_to_bag"),
max_camera_time_(-1)
{

   
}


void ROSDataLoader::load_data(const std::string &path_to_bag, std::vector<std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>>> &msg_groups_ready_for_process){

    this->path_to_bag_ = path_to_bag;
    this->bag_.open(this->path_to_bag_, rosbag::bagmode::Read);

    this->view_full_.addQuery(this->bag_);
    ros::Time rosbag_start_time = this->view_full_.getBeginTime();
    ros::Time rosbag_end_time = this->view_full_.getEndTime();
    VLOG(VERBOSE) << "ros bag start from " << std::to_string(rosbag_start_time.toSec()) << "s" << " to " << std::to_string(rosbag_end_time.toSec()) << "s";


    // Check to make sure we have data to play
    if (this->view_full_.size() == 0) {
        VLOG(VERBOSE) << RED << "[SERIAL]: No messages to play on specified topics.  Exiting." << RESET;
        ros::shutdown();
        std::exit(EXIT_FAILURE);
    }


    for (const rosbag::MessageInstance &msg : this->view_full_){
        if (!ros::ok()){
            break;
        }
        
        for (int cam_id = 0; cam_id < this->sys_config_->params_->max_cameras_; cam_id++) {
            
            const std::string &msg_topic = msg.getTopic();
            if (msg_topic ==  this->sys_config_->camera_config_->params_vector_.at(cam_id)->rgb_rostopic_) {
                this->loaded_msgs_.push_back(msg);
              this->max_camera_time_ = std::max(this->max_camera_time_, msg.getTime().toSec());
            }

            if (msg_topic ==  this->sys_config_->camera_config_->params_vector_.at(cam_id)->sensor_depth_rostopic_) {
                this->loaded_msgs_.push_back(msg);
                this->max_camera_time_ = std::max(this->max_camera_time_, msg.getTime().toSec());
              }


        }
            
    }

    VLOG(VERBOSE) << GREEN << "Total of " << loaded_msgs_.size() << " messages!" << RESET;



    std::unordered_set<int> used_index_set;
    // loop over all the message we have collected, synchronize them according to ros time
    for(size_t m=0; m < this->loaded_msgs_.size(); m++){

        VLOG(VERBOSE) << "we are preparing " << m << " message";

        // Skip messages that we have already used
        if (used_index_set.find(m) != used_index_set.end()) {
            used_index_set.erase(m);
            continue;
        }

        // we process camera data based on cam id = 0
        int base_cam_id = 0;
        std::string base_cam_rgb_rostopic = this->sys_config_->camera_config_->params_vector_.at(base_cam_id)->rgb_rostopic_;

        if (this->loaded_msgs_.at(m).getTopic() != base_cam_rgb_rostopic){
            continue;
        }

        // here we use shared_ptr just because map couldn't store value before initialization of rosbag::MessageInstance, 
        // where intialization process is very slow.
        std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>> msg_group;

        for(int cam_id=0; cam_id < this->sys_config_->params_->max_cameras_; cam_id++){

            std::map<std::string, std::shared_ptr<rosbag::MessageInstance>> dtype_to_msg_ptr_map;
            std::map<std::string, int> dtype_to_msg_idx_map;

            // base_cam_id message has been added in dtype_to_msg_idx_map
            if(cam_id == base_cam_id){
                dtype_to_msg_idx_map["bgr"] = m;
            }
            else{

                // find bgr according to base cam index, which m is base cam index.
                std::string cam_rgb_rostopic = this->sys_config_->camera_config_->params_vector_.at(cam_id)->rgb_rostopic_;
                int synchronized_idx = findSynchronizedIndex(cam_rgb_rostopic, m, this->sys_config_->params_->max_stereo_time_offset_);
                
                if (synchronized_idx != -1) {
                    dtype_to_msg_idx_map["bgr"] = synchronized_idx;
                    // we need used index for the image for stere pair, but no need for m, as it has been used already
                    used_index_set.insert(dtype_to_msg_idx_map["bgr"]);
                }
                else{
                    continue;
                }
            }
            
            dtype_to_msg_ptr_map["bgr"] = std::make_shared<rosbag::MessageInstance>(this->loaded_msgs_.at(dtype_to_msg_idx_map["bgr"]));
            
            // find sensor depth according to cam index which could be get from dtype_to_msg_idx_map
            if(this->sys_config_->camera_config_->params_vector_.at(cam_id)->use_sensor_depth_){

                std::string sensor_depth_rostopic = this->sys_config_->camera_config_->params_vector_.at(cam_id)->sensor_depth_rostopic_;
                int synchronized_idx = findSynchronizedIndex(sensor_depth_rostopic, dtype_to_msg_idx_map["bgr"], this->sys_config_->params_->max_color_sensor_depth_pair_time_offset_);

                if (synchronized_idx != -1) {
                    dtype_to_msg_idx_map["depth"] = synchronized_idx;
                    dtype_to_msg_ptr_map["depth"] = std::make_shared<rosbag::MessageInstance>(this->loaded_msgs_.at(synchronized_idx));
                    used_index_set.insert(dtype_to_msg_idx_map["depth"]);
                }
                else{
                    continue;                   
                }

            }

            msg_group.emplace_back(dtype_to_msg_ptr_map);
        }


        if(msg_group.size() == this->sys_config_->params_->max_cameras_){
            msg_groups_ready_for_process.emplace_back(msg_group);
        }
        else{
            VLOG(VERBOSE) << YELLOW <<"the message :" << m << " with rostopic :" << base_cam_rgb_rostopic \
            << ", unable to find all the pair, thus it will be discarded" << RESET;
        }

    }


    VLOG(VERBOSE) << GREEN << "we get "<< msg_groups_ready_for_process.size() << " message groups in total of " << loaded_msgs_.size() << " messages!" << RESET;



}






// we assume our ros data are chronological
int ROSDataLoader::findSynchronizedIndex(const std::string &target_rostopic, int m_source_index, double max_tolerant_time_offset){

    // mdi means index of base message(m) plus differentiate(d) index(i)
    int synchronized_idx = -1;
    double smallest_time_offset = std::numeric_limits<double>::infinity();
    int smallest_time_offset_idx = -1;
    double base_time = this->loaded_msgs_.at(m_source_index).getTime().toSec();
    double target_rostopic_time = -1;
    const std::string base_topic = this->loaded_msgs_.at(m_source_index).getTopic();

    for(int m_idx = 0; m_idx < (int)this->loaded_msgs_.size(); m_idx++){

        const std::string & rostopic = this->loaded_msgs_.at(m_idx).getTopic();
        // we found another message, m_mdi ,starting from m for another camera, cam_id.
        if(rostopic != target_rostopic){
            continue;
        }

        // compare time off between base message and another message(paired message)
        double base_time_plus_dt = this->loaded_msgs_.at(m_idx).getTime().toSec();
        double time_offset = std::abs(base_time_plus_dt-base_time);

        if(time_offset < smallest_time_offset){
            smallest_time_offset = time_offset;
            smallest_time_offset_idx = m_idx;
            target_rostopic_time = base_time_plus_dt;
        }
        // if tiem_offset in next index is greather than previous one then we don't have to look at the rest index anymore
        // because msgs are sorted by chronological order. so here we break the loop directly
        else{
            break;
        }
    }

    if(smallest_time_offset < max_tolerant_time_offset){
        synchronized_idx = smallest_time_offset_idx;
    }
    else{
        VLOG(VERBOSE) << YELLOW <<"with base rostopic: " << base_topic << "message :" << m_source_index << " in ros time : "<< std::to_string(base_time)  << " s "\
        << ", Unable to find the pair with rostopic:"<< target_rostopic <<". finally find message: " \
        <<  smallest_time_offset_idx << " in ros time : "<< std::to_string(target_rostopic_time)  << " s " << " , with the smallest time offset : " \
        << std::to_string(smallest_time_offset);     
    }


    return synchronized_idx;
}





    
} // namespace modules_vins




