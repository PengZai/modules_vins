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
        VLOG(VERBOSE) << RED << " maxCameras could not be out of configure number of camera" << RESET;
        std::exit(EXIT_FAILURE);
    }

    cv::FileNodeIterator it = camera_config->file_storage_->root().begin();
    for(int i = 0; i < this->params_->max_cameras_; i++, it++){
        
        std::shared_ptr<CameraParameters> camera_params = std::make_shared<CameraParameters>();
        std::string node_name = (*it).name();
        camera_params->loadFromNode(std::make_shared<cv::FileNode>((*camera_config->file_storage_)[node_name]));
        camera_config->params_vector_.emplace_back(camera_params);
        
    }


    camera_config->calculateExtrinsicsAndProjectionMatrixBetweenCameras();

    // load visualizer configs and their parameters
    std::string visualizer_config_path = relative_folder + this->params_->visualizer_config_name_;
    std::shared_ptr<VisualizerConfig> visualizer_config = std::make_shared<VisualizerConfig>();
    setVisualizerConfig(visualizer_config);
    visualizer_config->loadConfigFromPath(visualizer_config_path);


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


void SystemParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){
    
    
    this->node_ = node;
    parse("log_verbosity", this->log_verbosity_);
    parse("max_cameras", this->max_cameras_);
    parse("max_imus", this->max_imus_);
    
    parse("max_stereo_time_offset", this->max_stereo_time_offset_);
    parse("max_color_sensor_depth_pair_time_offset", this->max_color_sensor_depth_pair_time_offset_);

    parse("num_feature_points", this->num_feature_points_);
    parse("scale_factor", this->scale_factor_);
    parse("level_pyramid", this->level_pyramid_);

    parse("threshold_for_tracking_descriptor_in_time", this->threshold_for_tracking_descriptor_in_time_);
    parse("threshold_for_tracking_descriptor_in_frame", this->threshold_for_tracking_descriptor_in_frame_);
    parse("matching_ratio", this->matching_ratio_);


    parse("threshold_for_pnp_pose_log_norm", this->threshold_for_pnp_pose_log_norm_);
    parse("min_inliers", this->min_inliers_);
    parse("max_num_fail", this->max_num_fail_);
    parse("max_num_backward_reference", this->max_num_backward_reference_);

    
    parse("max_num_local_map_size", this->max_num_local_map_size_);

    

    parse("imu_config_name", this->imu_config_name_);
    parse("camera_config_name", this->camera_config_name_);
    parse("visualizer_config_name", this->visualizer_config_name_);

    parse("model_path", this->model_path_);
    parse("output_dir", this->output_dir_);

    parse("max_tolerant_gt_time_offset", this->max_tolerant_gt_time_offset_);
    parse("groundtruth_path", this->groundtruth_path_);
    parse("T_cam_GT", this->T_cam_GT_);

    
    parse("check_triangulation", this->check_triangulation_);

   
    VLOG(VERBOSE) << "config file loaded";

}


} // namespace modules_vins
