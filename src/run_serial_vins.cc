#include <iostream>
#include <string>
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <glog/logging.h>
#include "log/logging.h"
#include "frontend/visual_frontend.h"
#include "system/vins_system.h"
#include "system/system_config.h"



int main(int argc, char* argv[]) {



    ros::init(argc, argv, "run_serial_vins");


    // Ensure we have a path, if the user passes it then we should use it
    std::string config_path = "unset_path_to_config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    modules_vins::System sys;


    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");   // ("~") means create a private NodeHandle which only could access params inside the node
    sys.setNodehandler(nh);

    
    nh->param<std::string>("config_path", config_path, config_path);
    // Check if file exists
    if (!boost::filesystem::exists(config_path)) {
        std::cout << config_path << " not found" << std::endl;
        return -1;
    }


    std::shared_ptr<cv::FileStorage> sys_config_file_storage = std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
    if (!sys_config_file_storage->isOpened()) {
        std::cout << config_path << " not couldn't be open" << std::endl;
        return -1;
    }
    
    std::string log_verbosity = (*sys_config_file_storage)["system"]["log_verbosity"];
    Logger::setLogger(argv, log_verbosity);


    std::shared_ptr<modules_vins::SystemConfig> sys_config = std::make_shared<modules_vins::SystemConfig>();
    sys_config->loadFromPath(config_path);


    sys.setConfig(sys_config);
    std::shared_ptr<modules_vins::VisualFrontend> visual_frontend = std::make_shared<modules_vins::VisualFrontend>(sys_config);
    sys.setVisualFrontend(visual_frontend);

    std::shared_ptr<modules_vins::Visualizer> visualizer = std::make_shared<modules_vins::Visualizer>(sys_config, nh);
    sys.setVisualizer(visualizer);


    std::string path_to_bag="unset_path_to_bag.bag";
    nh->param<std::string>("bag_path", path_to_bag, path_to_bag);

    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    rosbag::View view_full;

    view_full.addQuery(bag);
    ros::Time rosbag_start_time = view_full.getBeginTime();
    ros::Time rosbag_end_time = view_full.getEndTime();
    VLOG(VERBOSE) << "ros bag start from " << std::to_string(rosbag_start_time.toSec()) << "s" << " to " << std::to_string(rosbag_end_time.toSec()) << "s";


    // Check to make sure we have data to play
    if (view_full.size() == 0) {
        VLOG(VERBOSE) << RED << "[SERIAL]: No messages to play on specified topics.  Exiting." << RESET;
        ros::shutdown();
        std::exit(EXIT_FAILURE);
    }

    double max_camera_time = -1;
    std::vector<rosbag::MessageInstance> msgs;
    for (const rosbag::MessageInstance &msg : view_full){
        if (!ros::ok()){
            break;
        }

        
        for (int cam_id = 0; cam_id < sys_config->params_->max_cameras_; cam_id++) {
            
            if (msg.getTopic() ==  sys_config->camera_config_->params_vector_.at(cam_id)->rostopic_) {
              msgs.push_back(msg);
              max_camera_time = std::max(max_camera_time, msg.getTime().toSec());
            }


        }
            
    }

    VLOG(VERBOSE) << "Total of " << msgs.size() << " messages!";

    
    std::unordered_set<int> used_index_set;
    // loop over all the message we have collected
    for(size_t m=0; m < msgs.size(); m++){

        // Skip messages that we have already used
        if (used_index_set.find(m) != used_index_set.end()) {
            used_index_set.erase(m);
            continue;
        }

        // moncular/stereo visual navigation callback
        sys.callbackVisualNavigation();

        // processing Camera data, then add them in deque
        for(int base_cam_id=0; base_cam_id < sys_config->params_->max_cameras_; base_cam_id++){
            // Skip if the topic of this message didn't same with camera topic
            std::string base_cam_rostopic = sys_config->camera_config_->params_vector_.at(base_cam_id)->rostopic_;
            if (msgs.at(m).getTopic() != base_cam_rostopic)
                continue;

            // here, topics in message and in camera config have been matched
            std::map<int, int> camid_to_msgid_map;
            double base_time = msgs.at(m).getTime().toSec();
            double largest_stereo_time_offset = -1;
            // we also need to find message coming from other cameras to pair or triple them in camid_to_msgid_map
            // but their time off between other cam and base_cam has to not be greater than MaxStereoTimeOff
            for(int cam_id=0; cam_id < sys_config->params_->max_cameras_; cam_id++){
                if(base_cam_id == cam_id){
                    camid_to_msgid_map.insert({cam_id, m});
                    continue;
                }
                // mdi means index of base message(m) plus differentiate(d) index(i)
                int mdi = -1;
                for(int m_mdi = m; m_mdi < (int)msgs.size(); m_mdi++){

                    // we found another message, m_mdi ,starting from m for another camera, cam_id.
                    std::string cam_rostopic = sys_config->camera_config_->params_vector_.at(cam_id)->rostopic_;
                    if(msgs.at(m_mdi).getTopic() != cam_rostopic){
                        continue;
                    }

                    // compare time off between base message and another message(paired message)
                    double base_time_plus_dt = msgs.at(m_mdi).getTime().toSec();
                    double stereo_time_offset = std::abs(base_time_plus_dt-base_time);
                    if(stereo_time_offset > largest_stereo_time_offset){
                        largest_stereo_time_offset = stereo_time_offset;
                    }
                    if(stereo_time_offset < sys_config->params_->max_stereo_time_offset_){
                        mdi = m_mdi;
                    }
                    // if next index is greather than threshold, then we don't have to look at the rest index anymore
                    // because msgs are sorted by chronological order. so here we break the loop directly
                    else{
                        break;
                    }
                }
                if(mdi != -1){
                    camid_to_msgid_map.insert({cam_id, mdi});
                }

            }

            // Skip processing if we were unable to find any messages
            if ((int)camid_to_msgid_map.size() != sys_config->params_->max_cameras_) {
                VLOG(VERBOSE) << YELLOW <<"with base cam rostopic: " << base_cam_rostopic << ", Unable to find stereo pair for message " <<  m \
                << " within the largest stereo time off " << largest_stereo_time_offset << " at time stamp" << base_time << " into bag (will skip!)\n" << RESET;

                continue;
            }

            std::vector<rosbag::MessageInstance> msgs_ready_for_deque;
            for(int i=0; i<(int)camid_to_msgid_map.size(); i++){
                msgs_ready_for_deque.emplace_back(msgs.at(camid_to_msgid_map.at(i)));
                used_index_set.insert(camid_to_msgid_map.at(i));
            }

            sys.addCameraFrameDeque(msgs_ready_for_deque);

        }



    }

    VLOG(VERBOSE) << GREEN << "vins has finished successfully, then close the programe" << RESET;


    ros::shutdown();

    return EXIT_SUCCESS;

}



