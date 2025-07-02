#pragma once


#include<memory>


#include "../system/system_config.h"
#include "../data/camera.h"
#include "../data/map.h"

namespace modules_vins{

class State{

    public:

        State(const std::shared_ptr<SystemConfig> sys_config);
        double getSynchronizedPoseTimestamp(const double base_timestamp, const double max_tolerant_time_offset, std::map<double, Sophus::SE3<double>> timestamp_comparison_T_full_map) const;
        bool getTransformationComparisonWorldWithIdx(const double base_timestamp, size_t used_idx, Sophus::SE3d &T_c_w);
        bool SynchronizeAndTransformComparisonPoseToRobotBaseCoordinate(const double base_timestamp);


        std::map<double, Sophus::SE3<double>> timestamp_T_b_w_map_; // the map of pose of robot in world coordinate
        std::map<double, Sophus::SE3<double>> timestamp_key_T_b_w_map_; // the map of pose of robot in world coordinate

        std::vector<std::map<double, Sophus::SE3<double>>> timestamp_T_w_comp_full_map_vector_; // a map of comparison pose of robot in world coordinate
        std::vector<std::map<double, Sophus::SE3<double>>> timestamp_T_comp_w_map_vector_; // a sub map of comparison pose of robot in world coordinate

     
        std::shared_ptr<SystemConfig> sys_config_;
        std::vector<Sophus::SE3<double>> T_imu_comparison_vector_;
        std::shared_ptr<Map> map_;

};


} //namespace modules_vins