#include "../log/logging.h"

#include <memory>


#include "config.h"

namespace modules_vins{


class OpenCVParameters: public Parameters{

    public:
    void loadFromNode(const std::shared_ptr<cv::FileNode> &node);

    public:
    int cv_waitkey_num_;
    bool show_projected_mappoint_;
    bool show_matching_in_frame_;
    bool show_matching_in_time_;
    bool show_tracking_in_time_;
    bool show_sensor_depth_;
    bool show_stereo_depth_;
    bool show_learned_depth_;
    bool show_learned_stereo_disparity_;
    bool show_object_detection_;
    bool show_semantic_segmentation_;

};


class RVisParameters: public Parameters{

    public:
    void loadFromNode(const std::shared_ptr<cv::FileNode> &node);

    public:

    std::string output_pose_rostopic_; 
    std::string output_key_frame_poses_rostopic_;
    std::string output_trajectory_rostopic_;
    std::string output_tracked_map_points_rostopic_; 
    bool show_groundtruth_trajectory_;
    std::string output_groundtruth_trajectory_rostopic_;
    bool show_groundtruth_pose_;
    std::string output_groundtruth_pose_rostopic_;
    
};

class PangolinParameters: public Parameters{

    public:
    void loadFromNode(const std::shared_ptr<cv::FileNode> &node);

    public:
    double frame_size_;
    double point_size_;
    double trajectory_line_size_;
    double viewer_eye_positionX_;
    double viewer_eye_positionY_;
    double viewer_eye_positionZ_;
    bool show_groundtruth_trajectory_;
    bool show_groundtruth_pose_;
};


class VisualizerParameters : public Parameters{


    public:
    void loadFromNode(const std::shared_ptr<cv::FileNode> &node);

    public:

    bool use_opencv_vis_;
    bool use_pangolin_vis_;
    bool use_rviz_vis_;


};

class VisualizerConfig : public Config{


    
    public:

    std::shared_ptr<VisualizerParameters> params_;

    std::shared_ptr<OpenCVParameters> opencv_params_;
    std::shared_ptr<RVisParameters> rviz_params_;
    std::shared_ptr<PangolinParameters> pangolin_params_;




};
    
    
} //modules_vins



