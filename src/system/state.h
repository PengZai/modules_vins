#pragma once


#include<memory>


#include "../system/system_config.h"
#include "../data/camera.h"

namespace modules_vins{

class State{

    public:

        double findSynchronizedPoseTimestamp(const double base_timestamp, const double max_tolerant_time_offset) const;
        bool SynchronizeAndTransformGTPoseWithTcw(const double base_timestamp, const double max_tolerant_time_offset, const Sophus::SE3<double> &T_cam_GT);


        std::map<double, Sophus::SE3<double>> timestamp_T_c_w_map_; // the map of pose of robot in world coordinate
        std::map<double, Sophus::SE3<double>> timestamp_GT_T_full_map_; // a map of Groundtruth pose of robot in world coordinate
        std::map<double, Sophus::SE3<double>> timestamp_GT_T_c_w_map_; // a map of Groundtruth pose of robot in world coordinate

     

        std::shared_ptr<Map> map_;

};


} //namespace modules_vins