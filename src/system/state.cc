#include "state.h"

namespace modules_vins{


    

bool State::SynchronizeAndTransformGTPoseWithTcw(const double base_timestamp, const double max_tolerant_time_offset, const Sophus::SE3<double> &T_cam_GT){

    double synchronized_gt_timestamp = findSynchronizedPoseTimestamp(base_timestamp, max_tolerant_time_offset);
    if(synchronized_gt_timestamp == -1){
        return false;
    }

    Sophus::SE3<double> GT_frist_pose_ = this->timestamp_GT_T_full_map_.at(synchronized_gt_timestamp);
    double GT_frist_timestamp_ = synchronized_gt_timestamp;

    auto it_start = this->timestamp_GT_T_full_map_.find(synchronized_gt_timestamp);
    auto it_end = this->timestamp_GT_T_full_map_.end();

    std::map<double, Sophus::SE3d> GT_T_sub_map(it_start, it_end);


    for (const auto& [timestamp, GT_T] : GT_T_sub_map) {

        this->timestamp_GT_T_c_w_map_[timestamp] = T_cam_GT * GT_frist_pose_.inverse() * GT_T;

    }
    
    return true;

}

double State::findSynchronizedPoseTimestamp(const double base_timestamp, const double max_tolerant_time_offset) const{

    double smallest_time_offset = std::numeric_limits<double>::infinity();
    double smallest_time_offset_timestamp = -1;
    double synchronized_timestamp = -1;

    for (const auto& [timestamp, GT_T_c_w] : this->timestamp_GT_T_full_map_) {


        double time_offset = std::abs(base_timestamp-timestamp);
        if(time_offset < smallest_time_offset){
            smallest_time_offset = time_offset;
            smallest_time_offset_timestamp = timestamp;
        }
        // if tiem_offset in next index is greather than previous one then we don't have to look at the rest index anymore
        // because msgs are sorted by chronological order. so here we break the loop directly
        else{
            break;
        }
    }

    if(smallest_time_offset < max_tolerant_time_offset){
        synchronized_timestamp = smallest_time_offset_timestamp; 
    }

    return synchronized_timestamp;
}


} //namespace modules_vins