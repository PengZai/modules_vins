#include <iostream>
#include <string>
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// #include "log/logging.h"
#include "frontend/visual_frontend.h"
#include "backend/keyframe_manager.h"
#include "system/vins_system.h"
#include "system/system_config.h"
#include "data/map.h"
#include "data/dataloader.h"
#include "log/evo_record.h"



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
    modules_vins::Logger::setLogger(argv, log_verbosity);


    std::shared_ptr<modules_vins::SystemConfig> sys_config = std::make_shared<modules_vins::SystemConfig>();
    sys_config->loadFromPath(config_path);


    sys.setConfig(sys_config);

    std::shared_ptr<modules_vins::Initializer> initializer = std::make_shared<modules_vins::Initializer>(sys_config);
    sys.setInitializer(initializer);

    std::shared_ptr<modules_vins::Map> map = std::make_shared<modules_vins::Map>(sys_config);
    sys.setMap(map);

    std::shared_ptr<modules_vins::EVORecorder> evo_recorder = std::make_shared<modules_vins::EVORecorder>(sys_config);
    sys.setRecorder(evo_recorder);

    std::shared_ptr<modules_vins::VisualFrontend> visual_frontend = std::make_shared<modules_vins::VisualFrontend>(sys_config);
    visual_frontend->setMap(map);
    sys.setVisualFrontend(visual_frontend);

    std::shared_ptr<modules_vins::KeyFrameManager> key_frame_manager = std::make_shared<modules_vins::KeyFrameManager>(sys_config);
    sys.setKeyFrameManager(key_frame_manager);

    std::shared_ptr<modules_vins::Visualizer> visualizer = std::make_shared<modules_vins::Visualizer>(sys_config, nh);
    visualizer->setMap(map);
    sys.setVisualizer(visualizer);


    std::shared_ptr<modules_vins::ROSDataLoader> ros_dataloader = std::make_shared<modules_vins::ROSDataLoader>(sys_config, nh);

    std::string path_to_bag = "path_to_bag";
    std::vector<std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>>> msg_groups_ready_for_process;
    nh->param<std::string>("bag_path", path_to_bag, path_to_bag);
    ros_dataloader->load_data(path_to_bag, msg_groups_ready_for_process);

    std::shared_ptr<modules_vins::FileDataLoader> file_dataloader = std::make_shared<modules_vins::FileDataLoader>(sys_config);

    std::map<double, Sophus::SE3<double>> timestamp_GT_T_full_map;
    file_dataloader->load_groundtruth(sys_config->params_->groundtruth_path_, timestamp_GT_T_full_map);
    sys.setGTState(timestamp_GT_T_full_map);

    for(size_t i=0; i < msg_groups_ready_for_process.size(); i++){

        const std::vector<std::map<std::string, std::shared_ptr<rosbag::MessageInstance>>> &msg_group = msg_groups_ready_for_process.at(i);
        sys.addCameraFrameDeque(msg_group);

        sys.callbackVisualNavigation();


    }

    LOG(INFO) << GREEN << "vins has finished successfully, then close the programe" << RESET;


    ros::shutdown();

    return EXIT_SUCCESS;

}



