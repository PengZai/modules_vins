#include "visualizer_config.h"


namespace modules_vins{


void OpenCVParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;



    parse("show_projected_mappoint", this->show_projected_mappoint_);

    parse("show_matching_in_frame", this->show_matching_in_frame_);
    parse("show_matching_in_time", this->show_matching_in_time_);
    
    parse("show_tracking_in_time", this->show_tracking_in_time_);
    parse("show_sensor_depth", this->show_sensor_depth_);
    parse("show_learned_depth", this->show_learned_depth_);
    parse("show_object_detection", this->show_object_detection_);
    parse("show_semantic_segmentation", this->show_semantic_segmentation_);

    
}



void RVisParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

    parse("output_pose_rostopic", this->output_pose_rostopic_);
    parse("output_trajectory_rostopic", this->output_trajectory_rostopic_);
    parse("output_tracked_map_points_rostopic", this->output_tracked_map_points_rostopic_);

    parse("show_groundtruth_trajectory", this->show_groundtruth_trajectory_);
    parse("output_groundtruth_trajectory_rostopic", this->output_groundtruth_trajectory_rostopic_);
    parse("show_groundtruth_pose", this->show_groundtruth_pose_);
    parse("output_groundtruth_pose_rostopic", this->output_groundtruth_pose_rostopic_);

  

}


void PangolinParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;


    parse("frame_size", this->frame_size_);
    parse("point_size", this->point_size_);
    parse("trajectory_line_size", this->trajectory_line_size_);
    parse("viewer_eye_positionX", this->viewer_eye_positionX_);
    parse("viewer_eye_positionY", this->viewer_eye_positionY_);
    parse("viewer_eye_positionZ", this->viewer_eye_positionZ_);

    parse("show_groundtruth_trajectory", this->show_groundtruth_trajectory_);
    parse("show_groundtruth_pose", this->show_groundtruth_pose_);

}

void VisualizerParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;


    parse("use_opencv_vis", this->use_opencv_vis_);
    parse("use_pangolin_vis", this->use_pangolin_vis_);
    parse("use_rviz_vis", this->use_rviz_vis_);
}






} //modules_vins