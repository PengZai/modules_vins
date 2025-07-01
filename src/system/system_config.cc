#include "system_config.h"

namespace modules_vins
{
    
SystemConfig::SystemConfig(){

    this->params_ = std::make_shared<SystemParameters>();
}


void SystemConfig::loadFromPath(const std::string &config_path){

    loadConfigFromPath(config_path);
    this->params_->loadFromNode(std::make_shared<cv::FileNode>((*this->file_storage_)["system"]));

    std::string relative_folder = this->path_.substr(0, this->path_.find_last_of('/')) + "/";

    // load camera configs and their parameters
    std::string camera_config_path = relative_folder + this->params_->camera_config_name_;
    std::shared_ptr<CameraConfig> camera_config = std::make_shared<CameraConfig>();
    setCameraConfig(camera_config);
    camera_config->loadConfigFromPath(camera_config_path);
    int Nnode = camera_config->file_storage_->root().size();
    if(Nnode < this->params_->max_cameras_){
        LOG(INFO) << RED << " maxCameras could not be out of configure number of camera" << RESET;
        std::exit(EXIT_FAILURE);
    }

    cv::FileNodeIterator camera_params_it = camera_config->file_storage_->root().begin();
    for(size_t i= 0; i < this->params_->max_cameras_; i++, camera_params_it++){
        
        std::shared_ptr<CameraParameters> camera_params = std::make_shared<CameraParameters>();
        std::string node_name = (*camera_params_it).name();
        camera_params->loadFromNode(std::make_shared<cv::FileNode>((*camera_config->file_storage_)[node_name]));
        camera_config->params_vector_.emplace_back(camera_params);
        
    }


    camera_config->calculateExtrinsicsAndProjectionMatrixBetweenCameras();

    // load benchmark configs and parameters
    std::string comparison_config_path = relative_folder + this->params_->comparison_config_name_;
    std::shared_ptr<ComparisonConfig> comparison_config = std::make_shared<ComparisonConfig>();
    setComparisonConfig(comparison_config);
    comparison_config->loadConfigFromPath(comparison_config_path);

    for (cv::FileNodeIterator comparison_params_it = comparison_config->file_storage_->root().begin(); comparison_params_it != comparison_config->file_storage_->root().end(); ++comparison_params_it) {

        std::shared_ptr<ComparisonParameters> comparison_params = std::make_shared<ComparisonParameters>();
        std::string node_name = (*comparison_params_it).name(); 
        comparison_params->setName(node_name);
        comparison_params->loadFromNode(std::make_shared<cv::FileNode>((*comparison_config->file_storage_)[node_name]));
        comparison_config->params_vector_.emplace_back(comparison_params);

        LOG(INFO) << GREEN << "loaded comparison : " << node_name << RESET;
    }



    // load feature and tracker configs and their parameters
    std::string feature_and_tracker_config_path = relative_folder + this->params_->feature_and_tracker_config_name_;
    std::shared_ptr<FeatureAndTrackerConfig> feature_and_tracker_config = std::make_shared<FeatureAndTrackerConfig>();
    setFeatureAndTrackerConfig(feature_and_tracker_config);
    feature_and_tracker_config->loadConfigFromPath(feature_and_tracker_config_path);

    // load visualizer parameters
    std::shared_ptr<FeatureAndTrackerParameters> feature_and_tracker_params = std::make_shared<FeatureAndTrackerParameters>();
    feature_and_tracker_params->loadFromNode(std::make_shared<cv::FileNode>((*feature_and_tracker_config->file_storage_)["commond"]));
    feature_and_tracker_config->params_ = feature_and_tracker_params;

    // load opencv parameters
    std::shared_ptr<ORBParameters> orb_params = std::make_shared<ORBParameters>();
    orb_params->loadFromNode(std::make_shared<cv::FileNode>((*feature_and_tracker_config->file_storage_)["ORB"]));
    feature_and_tracker_config->orb_params_ = orb_params;

    // load opencv parameters
    std::shared_ptr<KLTParameters> klt_params = std::make_shared<KLTParameters>();
    klt_params->loadFromNode(std::make_shared<cv::FileNode>((*feature_and_tracker_config->file_storage_)["KLT"]));
    feature_and_tracker_config->klt_params_ = klt_params;

    // load visualizer configs and their parameters
    std::string visualizer_config_path = relative_folder + this->params_->visualizer_config_name_;
    std::shared_ptr<VisualizerConfig> visualizer_config = std::make_shared<VisualizerConfig>();
    setVisualizerConfig(visualizer_config);
    visualizer_config->loadConfigFromPath(visualizer_config_path);

    // load visualizer parameters
    std::shared_ptr<VisualizerParameters> visualizer_params = std::make_shared<VisualizerParameters>();
    visualizer_params->loadFromNode(std::make_shared<cv::FileNode>((*visualizer_config->file_storage_)["visualizer"]));
    visualizer_config->params_ = visualizer_params;

    // load opencv parameters
    std::shared_ptr<OpenCVParameters> opencv_params = std::make_shared<OpenCVParameters>();
    opencv_params->loadFromNode(std::make_shared<cv::FileNode>((*visualizer_config->file_storage_)["opencv_visualizer"]));
    visualizer_config->opencv_params_ = opencv_params;

    // load rviz parameters
    std::shared_ptr<RVisParameters> rviz_params = std::make_shared<RVisParameters>();
    rviz_params->loadFromNode(std::make_shared<cv::FileNode>((*visualizer_config->file_storage_)["rviz_visualizer"]));
    visualizer_config->rviz_params_ = rviz_params;

    // load pangolin parameters
    std::shared_ptr<PangolinParameters> pangolin_params = std::make_shared<PangolinParameters>();
    pangolin_params->loadFromNode(std::make_shared<cv::FileNode>((*visualizer_config->file_storage_)["pangolin_visualizer"]));
    visualizer_config->pangolin_params_ = pangolin_params;



}


void SystemConfig::setCameraConfig(const std::shared_ptr<CameraConfig> &camera_config){
    this->camera_config_ = camera_config;
}

void SystemConfig::setVisualizerConfig(const std::shared_ptr<VisualizerConfig> &visualizer_config){
    this->visualizer_config_ = visualizer_config;
}

void SystemConfig::setFeatureAndTrackerConfig(const std::shared_ptr<FeatureAndTrackerConfig> feature_and_tracker_config){
    this->feature_and_tracker_config_ = feature_and_tracker_config;
}

void SystemConfig::setComparisonConfig(const std::shared_ptr<ComparisonConfig> benchmark_config){
    this->comparison_config_ = benchmark_config;
}



void SystemParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){
    
    
    this->node_ = node;
    parse("log_verbosity", this->log_verbosity_);
    parse("max_cameras", this->max_cameras_);
    parse("max_imus", this->max_imus_);
    
    parse("max_stereo_time_offset", this->max_stereo_time_offset_);
    parse("max_color_sensor_depth_pair_time_offset", this->max_color_sensor_depth_pair_time_offset_);



    // initialized pose estimation
    parse("maximum_num_fail", this->maximum_num_fail_);
    parse("minimum_num_in_ref_camera_frame", this->minimum_num_in_ref_camera_frame_);
    parse("minimum_cumulative_translation", this->minimum_cumulative_translation_);

    parse("threshold_for_pnp_pose_log_norm", this->threshold_for_pnp_pose_log_norm_);
    parse("min_inliers", this->min_inliers_);
    parse("max_fail_num", this->max_fail_num_);
    parse("max_num_backward_reference", this->max_num_backward_reference_);


    //backend
    parse("minimum_key_camera_frame_translation", this->minimum_key_camera_frame_translation_);

    //map
    parse("max_num_local_map_size", this->max_num_local_map_size_);

    //depth
    parse("maximum_estimated_depth", this->maximum_estimated_depth_);
    parse("minimum_estimated_depth", this->minimum_estimated_depth_);

    parse("imu_config_name", this->imu_config_name_);
    parse("camera_config_name", this->camera_config_name_);
    parse("visualizer_config_name", this->visualizer_config_name_);
    parse("feature_and_tracker_config_name", this->feature_and_tracker_config_name_);
    parse("comparison_config_name", this->comparison_config_name_);

    parse("model_path", this->model_path_);

    parse("use_comparison_pose_for_pose_estimation", this->use_comparison_pose_for_pose_estimation_);
    parse("comparison_pose_idx_for_pose_estimation", this->comparison_pose_idx_for_pose_estimation_);

    parse("output_dir", this->output_dir_);

     

    
    parse("check_triangulation", this->check_triangulation_);

   
    LOG(INFO) << "config file loaded";

}


} // namespace modules_vins
